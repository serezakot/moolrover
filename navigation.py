"""
Модуль навигации — сенсорные функции ("глаза") лунохода.
Отвечает на вопросы: где цель, под каким углом, как далеко.
Решения НЕ принимает — это делает state_machine.py.
"""

import math

# Порог уверенности: детекции YOLO слабее этого значения игнорируются.
CONFIDENCE_THRESHOLD = 0.5

# Горизонтальный угол обзора камеры в градусах (Camera Module 3 Wide — 102°).
CAMERA_HFOV = 102.0

# Реальный диаметр додекаэдра (ребро 1.5 см → описанная сфера ~4.2 см).
TARGET_REAL_SIZE = 0.042  # метры


def _coords(box):
    """Первые четыре элемента детекции — координаты бокса."""
    return box[0], box[1], box[2], box[3]


def box_confidence(box):
    """Уверенность детекции. Для бокса без неё (4 элемента) — 1.0."""
    return float(box[4]) if len(box) > 4 else 1.0


def filter_confident(detections, threshold=CONFIDENCE_THRESHOLD):
    """Оставляет только детекции с уверенностью >= threshold."""
    return [d for d in detections if box_confidence(d) >= threshold]


def choose_target(detections, threshold=CONFIDENCE_THRESHOLD):
    """Выбирает ОДНУ цель — самую близкую (по размеру бокса) среди надёжных."""
    confident = filter_confident(detections, threshold)
    if not confident:
        return None

    def area(box):
        x1, y1, x2, y2 = _coords(box)
        return (x2 - x1) * (y2 - y1)

    return max(confident, key=area)


def target_offset(detections, frame_width, threshold=CONFIDENCE_THRESHOLD):
    """Смещение цели: 0=центр, -1=левый край, +1=правый. None если цели нет."""
    target = choose_target(detections, threshold)
    if target is None:
        return None

    x1, y1, x2, y2 = _coords(target)
    target_cx = (x1 + x2) / 2
    frame_cx = frame_width / 2
    return (target_cx - frame_cx) / (frame_width / 2)


def target_angle(detections, frame_width, hfov=CAMERA_HFOV,
                 threshold=CONFIDENCE_THRESHOLD):
    """Угол до цели в градусах: минус=слева, плюс=справа. None если цели нет."""
    offset = target_offset(detections, frame_width, threshold)
    if offset is None:
        return None

    half = math.radians(hfov / 2)
    angle_rad = math.atan(offset * math.tan(half))
    return math.degrees(angle_rad)


def target_distance(detections, frame_width, hfov=CAMERA_HFOV,
                    real_size=TARGET_REAL_SIZE,
                    threshold=CONFIDENCE_THRESHOLD):
    """
    Расстояние до цели в метрах (приблизительное).
    Точность ~10-20% — достаточно для 'далеко/средне/близко/пора собирать'.
    None если цели нет.
    """
    target = choose_target(detections, threshold)
    if target is None:
        return None

    x1, y1, x2, y2 = _coords(target)
    box_width_px = x2 - x1

    if box_width_px <= 0:
        return None

    half_hfov = math.radians(hfov / 2)
    focal_px = (frame_width / 2) / math.tan(half_hfov)

    return (real_size * focal_px) / box_width_px