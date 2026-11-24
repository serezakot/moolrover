"""
Авто-настройка камеры Moonrover под текущее освещение + проверка "синий ли бокс".

Задача: камера должна одинаково честно показывать ЯРКО-СИНИЙ (по нему YOLO ищет
образцы) при любом свете.

Почему не подходят простые варианты:
  * ручной ColourGains под одно освещение ломается при другом (и душит синий);
  * постоянный авто-баланс (AwbEnable=True) "уплывает": крупный синий образец в
    кадре заставляет камеру думать "сцена синит" и гасить синий именно тогда,
    когда он нужен.

Решение — AWB-LOCK: на старте даём камере свести авто-баланс и авто-экспозицию
по нейтральной сцене, читаем подобранные параметры и ФИКСИРУЕМ их. Цвет
становится правильным для текущего света и больше не дрейфует. При смене
освещения — вызываем калибровку заново (страница /calibrate в track.py).
"""

import time

import cv2
import numpy as np


# Диапазон "синего" в OpenCV-HSV (H: 0..180). Подгони под свой образец,
# глядя на значение "blue" в видеопотоке.
BLUE_LOWER = np.array((90, 80, 60))
BLUE_UPPER = np.array((130, 255, 255))


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


def blue_fraction(frame_bgr, box=None) -> float:
    """Доля синих пикселей во всём кадре или внутри bbox (x1,y1,x2,y2), 0..1.

    Чек для боксов YOLO: настоящий образец синий -> доля высокая; ложное
    срабатывание на не-синем -> доля низкая. Порог ~0.2 на практике.
    """
    if box is not None:
        # сначала вырезаем бокс, потом конвертируем — не гоняем весь кадр в HSV
        x1, y1, x2, y2 = (max(0, int(v)) for v in box)
        frame_bgr = frame_bgr[y1:y2, x1:x2]
        if frame_bgr.size == 0:
            return 0.0
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
    return float(mask.mean()) / 255.0
