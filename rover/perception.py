"""Связка зрения с поведением.

Превращает «сырые» детекции YOLO (как в track.py) в TargetView, переиспользуя
"мозг" navigation.py — тот же выбор цели и тот же порог уверенности, что и в
ветке про confidence threshold. Так слой «вижу» один и тот же и для оверлея в
track.py, и для автомата миссии.
"""

from __future__ import annotations

import navigation as nav

from .hardware import TargetView


def build_target_view(detections, frame_width, frame_height,
                      threshold=nav.CONFIDENCE_THRESHOLD) -> TargetView:
    """Из списка детекций (x1,y1,x2,y2[,conf]) собирает TargetView.

    Цель выбирается и фильтруется по уверенности средствами navigation.py.
    """
    target = nav.choose_target(detections, threshold)
    if target is None:
        return TargetView.none()

    x1, y1, x2, y2 = nav._coords(target)
    box_area = (x2 - x1) * (y2 - y1)
    frame_area = float(frame_width * frame_height) or 1.0

    offset = nav.target_offset(detections, frame_width, threshold) or 0.0
    bearing = nav.target_angle(detections, frame_width, threshold=threshold) or 0.0

    return TargetView(
        visible=True,
        offset=offset,
        closeness=box_area / frame_area,
        bearing_deg=bearing,
        confidence=nav.box_confidence(target),
    )
