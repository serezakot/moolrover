"""Симуляция мира лунохода — чтобы проверить всю логику без железа.

Здесь живут заглушки интерфейсов из hardware.py поверх простой 2D-физики:
привод реально двигает «робота» по сцене, дальномеры лучами видят круги-
препятствия, датчик границы срабатывает у края арены, а манипулятор при
захвате «собирает» ближайший образец (он исчезает со сцены).

Это позволяет прогнать mission.py end-to-end и убедиться, что связка
вижу→еду→собираю→выезжаю работает, ещё до подключения моторов.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import navigation as nav

from .geometry import Pose, clamp
from .hardware import (
    BoundarySensor,
    Drive,
    DriveCommand,
    ManipState,
    Manipulator,
    RangeSensor,
    RangeView,
    TargetSensor,
    TargetView,
)


@dataclass
class SimConfig:
    bounds: float = 5.0              # арена [-bounds, bounds] по обеим осям
    max_speed: float = 0.15          # м/тик при linear=1
    max_turn_deg: float = 12.0       # град/тик при angular=1
    robot_radius: float = 0.18
    hfov_deg: float = nav.CAMERA_HFOV
    max_view: float = 6.0            # дальность зрения, м
    capture_dist: float = 0.7        # ближе этого манипулятор берёт образец
                                     # (>= дистанции, на которой срабатывает COLLECT)
    close_k: float = 0.19            # калибровка closeness((k/d)^2)
    range_max: float = 3.0           # дальность дальномеров, м


@dataclass
class SimWorld:
    config: SimConfig = field(default_factory=SimConfig)
    rover: Pose = field(default_factory=Pose)
    targets: List[Tuple[float, float]] = field(default_factory=list)
    obstacles: List[Tuple[float, float, float]] = field(default_factory=list)  # x,y,r
    collected: int = 0

    # --- управление миром приводом ---
    def apply(self, command: DriveCommand) -> None:
        cfg = self.config
        c = command.clamped()
        self.rover.rotate(c.angular * cfg.max_turn_deg)
        dist = c.linear * cfg.max_speed
        # проба хода с проверкой столкновений: не въезжаем в препятствие
        nx = self.rover.x + dist * math.cos(math.radians(self.rover.theta))
        ny = self.rover.y + dist * math.sin(math.radians(self.rover.theta))
        if not self._blocked_point(nx, ny):
            self.rover.x, self.rover.y = nx, ny

    def _blocked_point(self, x: float, y: float) -> bool:
        rr = self.config.robot_radius
        for ox, oy, orad in self.obstacles:
            if math.hypot(x - ox, y - oy) < orad + rr:
                return True
        return False

    # --- зрение ---
    def nearest_target(self) -> Optional[Tuple[float, float, float]]:
        best = None
        for tx, ty in self.targets:
            d = self.rover.distance_to(tx, ty)
            if best is None or d < best[2]:
                best = (tx, ty, d)
        return best

    def try_capture(self) -> bool:
        """Забрать ближайший образец, если он в зоне захвата."""
        near = self.nearest_target()
        if near and near[2] <= self.config.capture_dist:
            self.targets.remove((near[0], near[1]))
            self.collected += 1
            return True
        return False


# --------------------------------------------------------------------------
class SimDrive(Drive):
    def __init__(self, world: SimWorld):
        self.world = world

    def set_velocity(self, command: DriveCommand) -> None:
        self.world.apply(command)

    def odometry(self) -> Pose:
        return self.world.rover.copy()


class SimManipulator(Manipulator):
    """Манипулятор-заглушка. Операции срабатывают сразу (state меняется
    синхронно), а захват реально «собирает» образец в мире."""

    def __init__(self, world: SimWorld):
        self.world = world
        self._state = ManipState.STOWED

    def lower(self) -> None:
        self._state = ManipState.LOWERED

    def grip(self) -> None:
        # HOLDING только при реальном захвате образца, иначе остаёмся опущенными
        self._state = ManipState.HOLDING if self.world.try_capture() else ManipState.LOWERED

    def lift(self) -> None:
        pass  # остаётся HOLDING

    def release(self) -> None:
        self._state = ManipState.STOWED

    def state(self) -> ManipState:
        return self._state


class SimTargetSensor(TargetSensor):
    """Зрение по миру с тем же порогом уверенности, что и YOLO-конвейер:
    далёкая/краевая цель даёт низкую уверенность и отбрасывается."""

    def __init__(self, world: SimWorld, threshold: float = nav.CONFIDENCE_THRESHOLD):
        self.world = world
        self.threshold = threshold

    def sense(self) -> TargetView:
        cfg = self.world.config
        near = self.world.nearest_target()
        if near is None:
            return TargetView.none()
        tx, ty, dist = near
        if dist > cfg.max_view:
            return TargetView.none()

        world_bearing = self.world.rover.bearing_to(tx, ty)  # CCW(+) = слева
        half_fov = cfg.hfov_deg / 2.0
        if abs(world_bearing) > half_fov:
            return TargetView.none()         # вне поля зрения

        # Конвенция navigation/миссии: вправо = плюс (как смещение в кадре).
        offset = clamp(-world_bearing / half_fov, -1.0, 1.0)
        bearing = -world_bearing
        closeness = clamp((cfg.close_k / max(dist, 1e-3)) ** 2, 0.0, 1.0)
        confidence = clamp(0.95 - 0.11 * dist - 0.5 * abs(offset), 0.0, 1.0)

        if confidence < self.threshold:      # порог уверенности
            return TargetView.none()
        return TargetView(True, offset, closeness, bearing, confidence)


class SimRangeSensor(RangeSensor):
    """Три луча (фронт/лево/право) до ближайших кругов-препятствий."""

    def __init__(self, world: SimWorld, sector_deg: float = 35.0):
        self.world = world
        self.sector = sector_deg

    def read(self) -> RangeView:
        return RangeView(
            left=self._ray(self.sector),
            front=self._ray(0.0),
            right=self._ray(-self.sector),
        )

    def _ray(self, rel_deg: float) -> float:
        w = self.world
        p = w.rover
        ang = math.radians(p.theta + rel_deg)
        dx, dy = math.cos(ang), math.sin(ang)
        best = w.config.range_max
        for ox, oy, orad in w.obstacles:
            # проекция центра на луч
            t = (ox - p.x) * dx + (oy - p.y) * dy
            if t < 0:
                continue
            cx, cy = p.x + t * dx, p.y + t * dy
            perp = math.hypot(ox - cx, oy - cy)
            if perp <= orad:
                hit = t - math.sqrt(max(0.0, orad ** 2 - perp ** 2))
                best = min(best, max(0.0, hit))
        return best


class SimBoundarySensor(BoundarySensor):
    """Граница арены: чем ближе/за краем «нос» лунохода, тем выше уровень."""

    def __init__(self, world: SimWorld, look_ahead: float = 0.4, margin: float = 0.6):
        self.world = world
        self.look_ahead = look_ahead
        self.margin = margin

    def level(self) -> float:
        w = self.world
        p = w.rover
        b = w.config.bounds
        fx = p.x + self.look_ahead * math.cos(math.radians(p.theta))
        fy = p.y + self.look_ahead * math.sin(math.radians(p.theta))
        # расстояние «носа» до ближайшего края
        dist_to_edge = b - max(abs(fx), abs(fy))
        if dist_to_edge <= 0:
            return 1.0
        if dist_to_edge >= self.margin:
            return 0.0
        return clamp(1.0 - dist_to_edge / self.margin, 0.0, 1.0)


# --------------------------------------------------------------------------
def build_demo_world() -> SimWorld:
    """Сцена для демонстрации: цель ПОЗАДИ лунохода (нужен поиск),
    препятствие сбоку, вторая цель — чтобы показать повтор цикла."""
    world = SimWorld(config=SimConfig())
    world.rover = Pose(0.0, 0.0, 0.0)           # смотрит в +X
    world.targets = [(-2.0, 0.3), (3.0, -1.0)]  # первая — за спиной
    world.obstacles = [(-1.1, -1.0, 0.3)]       # сбоку от пути
    return world
