"""Автономная связка лунохода: вижу → еду → собираю → выезжаю.

Слои:
    hardware    — интерфейсы железа (привод, манипулятор, датчики) и типы;
    perception  — детекции YOLO (navigation.py) -> TargetView;
    search      — систематический поиск цели, когда её не видно;
    mission     — конечный автомат миссии поверх navigation.py;
    uart_link   — протокол к STM32 (моторы/манипулятор) для реального железа;
    simulation  — заглушки + 2D-физика для прогона логики без железа.

Реальный луноход и симуляция отличаются только тем, какие реализации
интерфейсов из hardware.py подставлены в RoverMission.
"""

from .geometry import Pose
from .hardware import (
    DriveCommand,
    ManipState,
    RangeView,
    TargetView,
)
from .mission import MissionConfig, RoverMission, State, StepResult
from .search import SearchConfig, SearchPlanner

__all__ = [
    "Pose",
    "DriveCommand",
    "ManipState",
    "RangeView",
    "TargetView",
    "RoverMission",
    "MissionConfig",
    "State",
    "StepResult",
    "SearchPlanner",
    "SearchConfig",
]
