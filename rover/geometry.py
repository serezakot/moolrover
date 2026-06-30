"""Мелкая геометрия для навигации: углы, поза, ограничения."""

from __future__ import annotations

import math
from dataclasses import dataclass


def clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def wrap_deg(angle: float) -> float:
    """Приводит угол к диапазону (-180, 180]."""
    a = (angle + 180.0) % 360.0 - 180.0
    return a + 360.0 if a <= -180.0 else a


def angle_diff(target: float, current: float) -> float:
    """Кратчайший знаковый поворот от current к target, градусы."""
    return wrap_deg(target - current)


@dataclass
class Pose:
    """Поза лунохода на плоскости. theta — курс в градусах (0 = +X, против ч.с.)."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def advance(self, distance: float) -> None:
        """Сдвиг вперёд по текущему курсу."""
        r = math.radians(self.theta)
        self.x += distance * math.cos(r)
        self.y += distance * math.sin(r)

    def rotate(self, dtheta: float) -> None:
        self.theta = wrap_deg(self.theta + dtheta)

    def bearing_to(self, x: float, y: float) -> float:
        """Курсовой угол на точку (x, y) относительно текущего курса, градусы."""
        absolute = math.degrees(math.atan2(y - self.y, x - self.x))
        return angle_diff(absolute, self.theta)

    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(x - self.x, y - self.y)

    def copy(self) -> "Pose":
        return Pose(self.x, self.y, self.theta)
