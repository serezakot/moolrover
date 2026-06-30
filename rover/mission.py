"""Конечный автомат миссии лунохода: вижу → еду → собираю → выезжаю.

Поверх реактивного "мозга" navigation.py добавлены состояние и память, чтобы
получилась цельная автономная логика:

    SEARCH   — цели не видно: систематический поиск (см. search.py).
    APPROACH — цель видна: пропорционально подъезжаем, держа её в центре.
    COLLECT  — цель близко и по центру: стоп, опустить-схватить-поднять.
    DEPART   — отъезд от точки сбора, чтобы не задеть образец, и снова поиск.
    AVOID    — граница/препятствие у самого носа: отъезд и уход в свободную
               сторону (наивысший приоритет безопасности).

Автомат не управляет железом напрямую — он дёргает интерфейсы из hardware.py,
поэтому одинаково работает и на STM32 (uart_link), и в симуляции.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import navigation as nav

from .geometry import Pose, angle_diff, clamp
from .hardware import (
    STOP,
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
from .search import SearchPlanner


class State(Enum):
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    COLLECT = "COLLECT"
    DEPART = "DEPART"
    AVOID = "AVOID"


@dataclass
class MissionConfig:
    # подъезд
    kp_angular: float = 1.6          # усиление центрирования цели
    kp_linear: float = 1.2           # усиление сближения
    align_offset: float = 0.20       # |offset| меньше — считаем "по центру"
    min_forward: float = 0.2         # минимум хода, когда цель по центру
    collect_closeness: float = nav.COLLECT_AREA
    boundary_level: float = nav.BOUNDARY_LEVEL
    stop_distance: float = 0.35      # препятствие ближе этого по фронту
    lost_target_ticks: int = 6       # сколько терпеть потерю цели в подъезде

    # сбор (длительности шагов манипулятора, тиков)
    lift_ticks: int = 3

    # отъезд
    depart_back_ticks: int = 4
    depart_turn_deg: float = 120.0

    # уход от границы
    avoid_back_ticks: int = 3
    avoid_turn_angular: float = 0.8

    # счисление пути, если нет энкодеров (фолбэк-одометрия)
    max_speed: float = 0.15          # метров за тик при linear=1
    max_turn_deg: float = 12.0       # градусов за тик при angular=1


@dataclass
class StepResult:
    state: State
    command: DriveCommand
    target: TargetView
    boundary: float
    manip: ManipState
    note: str = ""
    collected: int = 0


class RoverMission:
    def __init__(self, drive: Drive, manipulator: Manipulator,
                 target_sensor: TargetSensor, range_sensor: RangeSensor,
                 boundary_sensor: BoundarySensor,
                 config: Optional[MissionConfig] = None,
                 planner: Optional[SearchPlanner] = None):
        self.drive = drive
        self.manip = manipulator
        self.target_sensor = target_sensor
        self.range_sensor = range_sensor
        self.boundary_sensor = boundary_sensor
        self.cfg = config or MissionConfig()
        self.planner = planner or SearchPlanner()

        self.state = State.SEARCH
        self._prev_state: Optional[State] = None
        self.pose = Pose()
        self.last_seen_bearing: Optional[float] = None
        self.collected = 0

        self._lost = 0
        # под-счётчики секвенций
        self._collect_phase = 0
        self._collect_wait = 0
        self._collect_tries = 0
        self._depart_back = 0
        self._depart_turned = 0.0
        self._depart_ref: Optional[float] = None
        self._avoid_back = 0

    # ==================================================================
    def step(self) -> StepResult:
        """Один такт миссии: воспринять → решить → исполнить."""
        target = self.target_sensor.sense()
        boundary = self.boundary_sensor.level()
        ranges = self.range_sensor.read()

        odo = self.drive.odometry()
        if odo is not None:
            self.pose = odo

        if target.visible:
            self.last_seen_bearing = target.bearing_deg
            self._lost = 0

        command, note = self._policy(target, boundary, ranges)

        self.drive.set_velocity(command)
        if odo is None:                      # нет энкодеров — счисляем по команде
            self._integrate(command)

        return StepResult(self.state, command, target, boundary,
                          self.manip.state(), note, self.collected)

    # ==================================================================
    def _policy(self, target: TargetView, boundary: float,
                ranges: RangeView) -> tuple[DriveCommand, str]:
        cfg = self.cfg

        # --- незавершённые секвенции имеют приоритет (кроме безопасности) ---
        if self.state == State.COLLECT:
            return self._do_collect()
        if self.state == State.DEPART:
            return self._do_depart()
        if self.state == State.AVOID:
            return self._do_avoid(target, boundary, ranges)

        # --- безопасность: граница перебивает поиск/подъезд ---
        if boundary >= cfg.boundary_level:
            self._enter(State.AVOID)
            self._avoid_back = cfg.avoid_back_ticks
            return self._do_avoid(target, boundary, ranges)

        # --- цель видна? ---
        if target.visible:
            centered = abs(target.offset) <= cfg.align_offset
            if target.closeness >= cfg.collect_closeness and centered:
                self._enter(State.COLLECT)
                return self._do_collect()
            self._enter(State.APPROACH)
            return self._do_approach(target, ranges)

        # --- цель не видна: считаем потерю и идём в поиск ---
        if self._prev_state_was_tracking():
            self._lost += 1
            if self._lost < cfg.lost_target_ticks:
                # кратко до-крутиться к месту последнего контакта.
                # bearing>0 = цель была справа -> поворот вправо (angular<0)
                d = self.last_seen_bearing or 0.0
                turn = -1.0 if d > 0 else 1.0
                return DriveCommand(0.0, 0.5 * turn), "reacquire"

        self._enter_search()
        return self.planner.step(self.pose, ranges), "search"

    # ------------------------------------------------------------------
    def _prev_state_was_tracking(self) -> bool:
        return self.state == State.APPROACH

    def _enter_search(self) -> None:
        if self.state != State.SEARCH:
            self.planner.on_enter(self.last_seen_bearing)
        self._enter(State.SEARCH)

    def _enter(self, state: State) -> None:
        if state != self.state:
            self._prev_state = self.state
            self.state = state

    # --- ПОДЪЕЗД -------------------------------------------------------
    def _do_approach(self, target: TargetView, ranges: RangeView):
        cfg = self.cfg
        # центрируем цель: offset>0 (правее) -> поворот вправо (angular<0)
        angular = clamp(-cfg.kp_angular * target.offset, -1.0, 1.0)

        # сближение: чем дальше (меньше closeness), тем быстрее; притормаживаем
        # у цели и пока не выровнялись по курсу
        nearness = clamp(target.closeness / cfg.collect_closeness, 0.0, 1.0)
        linear = cfg.kp_linear * (1.0 - nearness)
        linear *= max(0.0, 1.0 - abs(target.offset))      # сначала довернуть
        linear = clamp(linear, 0.0, 1.0)
        if abs(target.offset) <= cfg.align_offset:
            linear = max(linear, cfg.min_forward)

        # препятствие на пути (и это не сама цель вплотную) — объехать
        if ranges.front_blocked(cfg.stop_distance) and nearness < 0.9:
            turn = 1.0 if ranges.clearest_side() == "left" else -1.0
            return DriveCommand(0.0, cfg.avoid_turn_angular * turn), "approach/obstacle"

        return DriveCommand(linear, angular), "approach"

    # --- СБОР ----------------------------------------------------------
    def _do_collect(self):
        """Стоп → опустить → схватить → (подтвердить) → поднять. Затем отъезд.
        Захват засчитывается только при подтверждённом HOLDING; неудачный
        захват несколько раз -> отмена сбора и возврат к подъезду/поиску."""
        self.drive.stop()
        phase = self._collect_phase

        if phase == 0:                       # опустить
            self.manip.lower()
            self._collect_phase = 1
            self._collect_tries = 0
            return STOP, "collect: lower"
        if phase == 1:                       # дождаться опускания, схватить
            if self.manip.state() == ManipState.LOWERED:
                self.manip.grip()
                self._collect_tries += 1
                self._collect_phase = 2
            return STOP, "collect: grip"
        if phase == 2:                       # подтвердить захват и поднять
            if self.manip.is_holding():
                self.collected += 1          # засчитываем только подтверждённый
                self.manip.lift()
                self._collect_wait = self.cfg.lift_ticks
                self._collect_phase = 3
                return STOP, "collect: lift"
            # захват не удался — повторить, иначе отменить сбор
            if self._collect_tries >= 3:
                self._collect_phase = 0
                self._enter(State.APPROACH)
                return STOP, "collect: failed"
            self._collect_phase = 1
            return STOP, "collect: retry"
        # phase 3: дать манипулятору подняться, затем отъезд
        self._collect_wait -= 1
        if self._collect_wait <= 0:
            self.manip.release()             # выгрузка образца в бункер
            self._collect_phase = 0
            self._begin_depart()
            return STOP, "collect: done"
        return STOP, "collect: lifting"

    # --- ОТЪЕЗД --------------------------------------------------------
    def _begin_depart(self) -> None:
        self._enter(State.DEPART)
        self._depart_back = self.cfg.depart_back_ticks
        self._depart_turned = 0.0
        self._depart_ref = None

    def _do_depart(self):
        cfg = self.cfg
        if self._depart_back > 0:            # отъехать назад от точки сбора
            self._depart_back -= 1
            return DriveCommand(-0.6, 0.0), "depart: back"

        if self._depart_ref is None:
            self._depart_ref = self.pose.theta
        self._depart_turned += abs(angle_diff(self.pose.theta, self._depart_ref))
        self._depart_ref = self.pose.theta
        if self._depart_turned >= cfg.depart_turn_deg:   # развернуться и искать дальше
            self._enter_search()
            return self.planner.step(self.pose, RangeView()), "depart: done"
        return DriveCommand(0.0, 0.8), "depart: turn"

    # --- УХОД ОТ ГРАНИЦЫ/ПРЕПЯТСТВИЯ -----------------------------------
    def _do_avoid(self, target: TargetView, boundary: float, ranges: RangeView):
        cfg = self.cfg
        if self._avoid_back > 0:             # сначала отъехать от кромки
            self._avoid_back -= 1
            return DriveCommand(-0.6, 0.0), "avoid: back"

        # пока опасно — доворачиваем в более свободную сторону
        if boundary >= cfg.boundary_level or ranges.front_blocked(cfg.stop_distance):
            turn = 1.0 if ranges.clearest_side() == "left" else -1.0
            return DriveCommand(0.0, cfg.avoid_turn_angular * turn), "avoid: turn"

        # безопасно — вернуться к делу
        if target.visible:
            self._enter(State.APPROACH)
            return self._do_approach(target, ranges)
        self._enter_search()
        return self.planner.step(self.pose, ranges), "avoid: clear"

    # ------------------------------------------------------------------
    def _integrate(self, command: DriveCommand) -> None:
        """Фолбэк-одометрия: обновить позу по выданной команде."""
        c = command.clamped()
        self.pose.rotate(c.angular * self.cfg.max_turn_deg)
        self.pose.advance(c.linear * self.cfg.max_speed)

    # ------------------------------------------------------------------
    def run(self, max_ticks: int = 2000, on_step=None) -> int:
        """Гонять миссию max_ticks тактов (для симуляции/тестов).
        Возвращает число собранных образцов."""
        for _ in range(max_ticks):
            result = self.step()
            if on_step is not None:
                on_step(result)
        return self.collected
