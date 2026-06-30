"""Тесты "мозга" навигации: порог уверенности и принятие решений.

Чистый Python — navigation.py зависит только от math, поэтому тесты
запускаются без YOLO/OpenCV/камеры:

    python3 -m pytest tests/test_navigation.py    # если есть pytest
    python3 tests/test_navigation.py              # как обычный скрипт
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import navigation as nav  # noqa: E402

W, H = 1280, 720


def box(cx, cy=360, size=60, conf=None):
    """Бокс с центром (cx, cy). conf=None -> 4-кортеж без уверенности."""
    half = size / 2
    coords = (cx - half, cy - half, cx + half, cy + half)
    return coords if conf is None else (*coords, conf)


# --- Порог уверенности ----------------------------------------------------

def test_low_confidence_is_ignored():
    dets = [box(W / 2, conf=0.2)]  # уверенность ниже порога 0.5
    assert nav.choose_target(dets) is None
    assert nav.decide(dets, W, H) == "SEARCH"


def test_high_confidence_is_used():
    dets = [box(W / 2, conf=0.9)]
    assert nav.choose_target(dets) is not None
    assert nav.decide(dets, W, H) == "FORWARD"


def test_threshold_boundary_inclusive():
    dets = [box(W / 2, conf=0.5)]  # ровно порог -> проходит
    assert nav.choose_target(dets) is not None


def test_backward_compatible_4tuple():
    # Старый формат без уверенности считается надёжным (conf=1.0).
    dets = [box(W / 2)]  # 4-кортеж
    assert nav.box_confidence(dets[0]) == 1.0
    assert nav.choose_target(dets) is not None


def test_confident_target_wins_over_bigger_noisy_one():
    # Большой, но неуверенный бокс не должен перебивать маленький уверенный.
    noisy_big = box(200, size=400, conf=0.2)
    small_sure = box(1000, size=60, conf=0.9)
    target = nav.choose_target([noisy_big, small_sure])
    assert target == small_sure


def test_custom_threshold_argument():
    dets = [box(W / 2, conf=0.4)]
    assert nav.choose_target(dets, threshold=0.3) is not None
    assert nav.choose_target(dets, threshold=0.6) is None


# --- Решения автомата -----------------------------------------------------

def test_boundary_overrides_everything():
    dets = [box(W / 2, conf=0.99)]
    assert nav.decide(dets, W, H, boundary_level=0.5) == "AVOID"


def test_search_when_empty():
    assert nav.decide([], W, H) == "SEARCH"


def test_collect_when_close():
    # бокс занимает > COLLECT_AREA доли кадра
    big = box(W / 2, size=int((W * H * 0.2) ** 0.5), conf=0.9)
    assert nav.decide([big], W, H) == "COLLECT"


def test_left_right_forward():
    assert nav.decide([box(100, conf=0.9)], W, H) == "LEFT"
    assert nav.decide([box(W - 100, conf=0.9)], W, H) == "RIGHT"
    assert nav.decide([box(W / 2, conf=0.9)], W, H) == "FORWARD"


def test_offset_and_angle_signs():
    left = [box(100, conf=0.9)]
    right = [box(W - 100, conf=0.9)]
    assert nav.target_offset(left, W) < 0
    assert nav.target_offset(right, W) > 0
    assert nav.target_angle(left, W) < 0
    assert nav.target_angle(right, W) > 0
    assert nav.target_offset([], W) is None


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"  ok  {fn.__name__}")
    print(f"\n{len(fns)} тестов пройдено")


if __name__ == "__main__":
    _run_all()
