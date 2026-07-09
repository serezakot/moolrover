"""
Авто-настройка камеры Moonrover под текущее освещение.

Задача: камера должна одинаково честно показывать ЯРКО-СИНИЙ (по нему YOLO ищет
образцы) и ЧЁРНЫЙ (по нему детектится граница).

Почему не подходят простые варианты:
  * ручной ColourGains под одно освещение ломается при другом (и душит синий);
  * постоянный авто-баланс (AwbEnable=True) "уплывает": крупный синий образец в
    кадре заставляет камеру думать "сцена синит" и гасить синий именно тогда,
    когда он нужен.

Решение — AWB-LOCK: на старте даём камере свести авто-баланс и авто-экспозицию
по нейтральной сцене, читаем подобранные параметры и ФИКСИРУЕМ их. Цвет
становится правильным для текущего света и больше не дрейфует. При смене
освещения — вызываем калибровку заново.

ВАЖНО: модуль работает с picamera2 на Raspberry Pi; на машине без камеры
функции цвета (gray_world / blue_fraction) можно тестировать на обычных кадрах
OpenCV. Проверять на самой Pi обязательно — параметры зависят от железа/света.
"""

import time

try:
    import cv2
    import numpy as np
except ImportError:  # цветовые функции недоступны без OpenCV/numpy
    cv2 = None
    np = None


# Диапазон "синего" в OpenCV-HSV (H: 0..180). Подгони под свой образец,
# глядя на blue_fraction в видеопотоке.
BLUE_LOWER = (90, 80, 60)
BLUE_UPPER = (130, 255, 255)


# --------------------------------------------------------------------------
#  1. Калибровка баланса белого / экспозиции под текущее освещение
# --------------------------------------------------------------------------
def auto_calibrate(picam2, settle_seconds: float = 2.0, verbose: bool = True):
    """Свести авто-баланс/экспозицию и зафиксировать их (AWB-lock).

    Наведи камеру на нейтральный фон (белый/серый лист) и вызови эту функцию
    один раз на старте. Возвращает словарь зафиксированных значений.
    """
    # 1) включаем авто-режимы
    picam2.set_controls({"AwbEnable": True, "AeEnable": True})
    # 2) даём камере свестись по сцене
    time.sleep(settle_seconds)
    # 3) читаем, что подобрала камера
    meta = picam2.capture_metadata()
    gains = meta.get("ColourGains")
    exposure = meta.get("ExposureTime")
    analogue = meta.get("AnalogueGain")
    # 4) фиксируем — дальше цвет и яркость стабильны
    locked = {"AwbEnable": False, "AeEnable": False}
    if gains:
        locked["ColourGains"] = (float(gains[0]), float(gains[1]))
    if exposure:
        locked["ExposureTime"] = int(exposure)
    if analogue:
        locked["AnalogueGain"] = float(analogue)
    picam2.set_controls(locked)
    if verbose:
        print(f"[camera] калибровка зафиксирована: {locked}")
    return locked


def recalibrate_on_demand(picam2):
    """Удобная обёртка: пере-калибровать (например, по кнопке/HTTP-команде,
    когда сменилось освещение)."""
    return auto_calibrate(picam2)


# --------------------------------------------------------------------------
#  2. Программный баланс белого "серый мир" (фолбэк, не зависит от железа)
# --------------------------------------------------------------------------
def gray_world(frame_bgr):
    """Выровнять средние по каналам (классический Gray-World).
    Возвращает цвето-скорректированный кадр. Полезно, если железному балансу
    нельзя доверять или модель обучалась на нейтральных кадрах."""
    if np is None:
        return frame_bgr
    f = frame_bgr.astype(np.float32)
    means = f.reshape(-1, 3).mean(axis=0)          # B, G, R
    gray = float(means.mean())
    scale = gray / np.clip(means, 1e-6, None)
    return np.clip(f * scale, 0, 255).astype(np.uint8)


# --------------------------------------------------------------------------
#  3. Проверка "реально ли это синий" по HSV (не зависит от баланса белого)
# --------------------------------------------------------------------------
def blue_mask(frame_bgr):
    """Маска синих пикселей кадра (по оттенку H, а не по RGB)."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, np.array(BLUE_LOWER), np.array(BLUE_UPPER))


def blue_fraction(frame_bgr, box=None) -> float:
    """Доля синих пикселей во всём кадре или внутри bbox (x1,y1,x2,y2), 0..1.

    Чек для боксов YOLO: настоящий образец синий -> доля высокая; ложное
    срабатывание на не-синем -> доля низкая. Порог ~0.2 на практике.
    """
    if cv2 is None:
        return 0.0
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    if box is not None:
        x1, y1, x2, y2 = (int(v) for v in box)
        hsv = hsv[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
        if hsv.size == 0:
            return 0.0
    mask = cv2.inRange(hsv, np.array(BLUE_LOWER), np.array(BLUE_UPPER))
    return float(mask.mean()) / 255.0


def is_blue_target(frame_bgr, box, min_fraction: float = 0.2) -> bool:
    """True, если внутри bbox достаточно синего — образец, а не ложь."""
    return blue_fraction(frame_bgr, box) >= min_fraction


# --------------------------------------------------------------------------
#  4. (опц.) Монитор освещения: подсказать, когда пора пере-калибровать
# --------------------------------------------------------------------------
class LightingMonitor:
    """Следит за средней яркостью кадра. Если она ушла далеко от той, что
    была на момент калибровки, советует пере-калибровать."""

    def __init__(self, tolerance: float = 35.0):
        self.reference = None
        self.tolerance = tolerance

    def set_reference(self, frame_bgr):
        self.reference = self._mean_luma(frame_bgr)

    def needs_recalibration(self, frame_bgr) -> bool:
        if self.reference is None:
            self.set_reference(frame_bgr)
            return False
        return abs(self._mean_luma(frame_bgr) - self.reference) > self.tolerance

    @staticmethod
    def _mean_luma(frame_bgr) -> float:
        if cv2 is None:
            return 0.0
        return float(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY).mean())
