"""
Модуль навигации ("мозг") лунохода.
Принимает координаты ресурсов от YOLO + уровень границы и решает, что делать.
Пока НЕ управляет моторами — только возвращает решение.
"""

import math

DEADZONE = 0.15        # доля ширины кадра по центру, где цель считаем "по центру"
COLLECT_AREA = 0.10    # если бокс больше этой доли площади кадра — цель близко, собираем
BOUNDARY_LEVEL = 0.15  # если доля "чёрного" перед луноходом >= этого — уходим от границы

# Горизонтальный угол обзора камеры в градусах (Camera Module 3 Wide — 102°).
CAMERA_HFOV = 102.0


def choose_target(detections):
    """Выбирает ОДНУ цель — самую близкую (по размеру бокса). Или None."""
    if not detections:
        return None

    def area(box):
        x1, y1, x2, y2 = box
        return (x2 - x1) * (y2 - y1)

    return max(detections, key=area)


def target_offset(detections, frame_width):
    """Смещение цели: 0=центр, -1=левый край, +1=правый. None если цели нет."""
    target = choose_target(detections)
    if target is None:
        return None

    x1, y1, x2, y2 = target
    target_cx = (x1 + x2) / 2
    frame_cx = frame_width / 2
    return (target_cx - frame_cx) / (frame_width / 2)


def target_angle(detections, frame_width, hfov=CAMERA_HFOV):
    """Угол до цели в градусах: минус=слева, плюс=справа. None если цели нет."""
    offset = target_offset(detections, frame_width)
    if offset is None:
        return None

    half = math.radians(hfov / 2)
    angle_rad = math.atan(offset * math.tan(half))
    return math.degrees(angle_rad)


def decide(detections, frame_width, frame_height, boundary_level=0.0):
    """
    Главная функция "мозга".
    boundary_level — доля тёмных пикселей перед луноходом (0..1).
    Возвращает: 'AVOID', 'SEARCH', 'LEFT', 'RIGHT', 'FORWARD' или 'COLLECT'.
    """
    # ГРАНИЦА — высший приоритет, перебивает всё остальное
    if boundary_level >= BOUNDARY_LEVEL:
        return "AVOID"

    target = choose_target(detections)
    if target is None:
        return "SEARCH"

    x1, y1, x2, y2 = target

    box_area = (x2 - x1) * (y2 - y1)
    frame_area = frame_width * frame_height
    closeness = box_area / frame_area

    if closeness >= COLLECT_AREA:
        return "COLLECT"

    target_cx = (x1 + x2) / 2
    frame_cx = frame_width / 2
    offset = target_cx - frame_cx
    dead_px = DEADZONE * frame_width

    if offset < -dead_px:
        return "LEFT"
    elif offset > dead_px:
        return "RIGHT"
    else:
        return "FORWARD"