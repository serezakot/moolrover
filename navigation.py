"""
Модуль навигации ("мозг") лунохода.
Принимает координаты ресурсов от YOLO + уровень границы и решает, что делать.
Пока НЕ управляет моторами — только возвращает решение.

Детекция теперь может нести уверенность YOLO:
    (x1, y1, x2, y2)            — без уверенности (считается надёжной, conf=1.0)
    (x1, y1, x2, y2, conf)      — с уверенностью YOLO в диапазоне [0..1]

Цели с уверенностью ниже CONFIDENCE_THRESHOLD отбрасываются ещё до выбора —
луноход не реагирует на ненадёжные срабатывания (блики, тени, шум модели).
"""

import math

DEADZONE = 0.15        # доля ширины кадра по центру, где цель считаем "по центру"
COLLECT_AREA = 0.10    # если бокс больше этой доли площади кадра — цель близко, собираем
BOUNDARY_LEVEL = 0.15  # если доля "чёрного" перед луноходом >= этого — уходим от границы

# Порог уверенности: детекции YOLO слабее этого значения игнорируются.
CONFIDENCE_THRESHOLD = 0.5

# Горизонтальный угол обзора камеры в градусах (Camera Module 3 Wide — 102°).
CAMERA_HFOV = 102.0


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
    """Выбирает ОДНУ цель — самую близкую (по размеру бокса) среди надёжных.
    Возвращает None, если уверенных детекций нет."""
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


def decide(detections, frame_width, frame_height, boundary_level=0.0,
           confidence_threshold=CONFIDENCE_THRESHOLD):
    """
    Главная функция "мозга".
    boundary_level — доля тёмных пикселей перед луноходом (0..1).
    confidence_threshold — порог уверенности для детекций YOLO.
    Возвращает: 'AVOID', 'SEARCH', 'LEFT', 'RIGHT', 'FORWARD' или 'COLLECT'.
    """
    # ГРАНИЦА — высший приоритет, перебивает всё остальное
    if boundary_level >= BOUNDARY_LEVEL:
        return "AVOID"

    target = choose_target(detections, confidence_threshold)
    if target is None:
        # Надёжной цели нет (или все ниже порога) — переходим к поиску.
        return "SEARCH"

    x1, y1, x2, y2 = _coords(target)

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
