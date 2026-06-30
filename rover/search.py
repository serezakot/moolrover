"""Планировщик поиска цели.

Когда надёжной цели в кадре нет, нельзя крутиться на месте или метаться
случайно. Поиск построен систематически и от быстрого к широкому:

  1. REACQUIRE — если цель только что потеряли, сначала доворачиваемся туда,
     где её видели в последний раз (самый быстрый способ вернуть контакт).
  2. SCAN      — полный круговой обзор на месте: медленно крутимся на 360°,
     давая детектору рассмотреть всё вокруг.
  3. RELOCATE  — если на месте пусто, переезжаем на новую точку обзора по
     расширяющейся спирали (boustrophedon-подобное покрытие), объезжая
     препятствия по дальномерам, и снова делаем круговой обзор.

Память: накопленный поворот меряется по реальной одометрии (позе), а
пройденные клетки запоминаются — поиск расширяется наружу, а не топчется.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Set, Tuple

from .geometry import Pose, angle_diff
from .hardware import DriveCommand, RangeView


@dataclass
class SearchConfig:
    scan_angular: float = 0.7        # скорость вращения при круговом обзоре
    scan_full_deg: float = 360.0     # сколько прокрутить, чтобы счесть место осмотренным
    reacquire_margin_deg: float = 20.0
    forward_linear: float = 0.7      # скорость при переезде
    leg_ticks: int = 6               # длина первого «отрезка» спирали, тиков
    leg_grow_ticks: int = 3          # на сколько удлинять отрезок каждые 2 поворота
    turn_angular: float = 0.8        # скорость доворота на новой точке
    turn_target_deg: float = 90.0    # угол поворота спирали
    stop_distance: float = 0.35      # ближе этого по фронту — препятствие
    cell_size: float = 0.5           # размер клетки памяти, метры


@dataclass
class SearchPlanner:
    config: SearchConfig = field(default_factory=SearchConfig)

    mode: str = "SCAN"               # SCAN | RELOCATE | REACQUIRE
    _scanned: float = 0.0
    _spin_dir: float = 1.0
    _last_theta: Optional[float] = None

    # реакквизиция
    _reacquire_left: float = 0.0

    # релокация (спираль)
    _relocate_phase: str = "forward"  # forward | turn
    _leg_left: int = 0
    _leg_len: int = 0
    _turned: float = 0.0
    _legs_done: int = 0
    _spiral_dir: float = 1.0
    _blocked_ticks: int = 0

    visited: Set[Tuple[int, int]] = field(default_factory=set)

    # ------------------------------------------------------------------
    def on_enter(self, last_seen_bearing: Optional[float]) -> None:
        """Вызывается при входе в режим поиска. Если цель только что
        потеряли — начинаем с доворота к месту последнего контакта."""
        self._last_theta = None
        if last_seen_bearing is not None and abs(last_seen_bearing) > 1.0:
            self.mode = "REACQUIRE"
            # bearing>0 = цель была справа -> доворот вправо (angular<0)
            self._spin_dir = -1.0 if last_seen_bearing > 0 else 1.0
            self._reacquire_left = abs(last_seen_bearing) + self.config.reacquire_margin_deg
        else:
            self._start_scan()

    # ------------------------------------------------------------------
    def step(self, pose: Pose, ranges: RangeView) -> DriveCommand:
        """Одна команда движения для поиска."""
        moved = self._consume_rotation(pose)
        self._mark_visited(pose)

        if self.mode == "REACQUIRE":
            return self._step_reacquire(moved)
        if self.mode == "SCAN":
            return self._step_scan(moved)
        return self._step_relocate(ranges, moved)

    # ------------------------------------------------------------------
    def _consume_rotation(self, pose: Pose) -> float:
        """Сколько градусов реально повернулись с прошлого тика (по одометрии)."""
        if self._last_theta is None:
            self._last_theta = pose.theta
            return 0.0
        d = abs(angle_diff(pose.theta, self._last_theta))
        self._last_theta = pose.theta
        return d

    def _mark_visited(self, pose: Pose) -> None:
        cs = self.config.cell_size
        self.visited.add((round(pose.x / cs), round(pose.y / cs)))

    # --- режимы --------------------------------------------------------
    def _start_scan(self) -> None:
        self.mode = "SCAN"
        self._scanned = 0.0

    def _step_reacquire(self, moved: float) -> DriveCommand:
        self._reacquire_left -= moved
        if self._reacquire_left <= 0:
            self._start_scan()
            return self._step_scan(0.0)
        return DriveCommand(0.0, self.config.scan_angular * self._spin_dir)

    def _step_scan(self, moved: float) -> DriveCommand:
        self._scanned += moved
        if self._scanned >= self.config.scan_full_deg:
            self._start_relocate()
            return self._step_relocate(RangeView(), 0.0)
        return DriveCommand(0.0, self.config.scan_angular * self._spin_dir)

    def _start_relocate(self) -> None:
        self.mode = "RELOCATE"
        self._relocate_phase = "forward"
        if self._leg_len == 0:
            self._leg_len = self.config.leg_ticks
        self._leg_left = self._leg_len
        self._turned = 0.0
        self._blocked_ticks = 0

    def _step_relocate(self, ranges: RangeView, moved: float) -> DriveCommand:
        cfg = self.config

        if self._relocate_phase == "forward":
            # Объезд: если фронт занят — поворачиваем в более свободную сторону.
            if ranges.front_blocked(cfg.stop_distance):
                self._blocked_ticks += 1
                turn = 1.0 if ranges.clearest_side() == "left" else -1.0
                # застряли надолго — заканчиваем отрезок, пойдём в обзор
                if self._blocked_ticks > 8:
                    self._begin_relocate_turn()
                return DriveCommand(0.0, cfg.turn_angular * turn)

            self._blocked_ticks = 0
            self._leg_left -= 1
            if self._leg_left <= 0:
                self._begin_relocate_turn()
            return DriveCommand(cfg.forward_linear, 0.0)

        # phase == "turn": доворот спирали на ~90°
        self._turned += moved
        if self._turned >= cfg.turn_target_deg:
            self._legs_done += 1
            if self._legs_done % 2 == 0:                 # каждые 2 ноги — шире шаг
                self._leg_len += cfg.leg_grow_ticks
            self._start_scan()                            # на новой точке — снова обзор
            return self._step_scan(0.0)
        return DriveCommand(0.0, cfg.turn_angular * self._spiral_dir)

    def _begin_relocate_turn(self) -> None:
        self._relocate_phase = "turn"
        self._turned = 0.0
