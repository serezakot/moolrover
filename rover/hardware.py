"""Абстракции железа лунохода.

Здесь только интерфейсы и типы данных — никакой реализации под конкретное
железо. Это «контракт», к которому потом подключаются настоящие моторы,
манипулятор и датчики (через UART к STM32) — см. uart_link.py, а для отладки
логики — заглушки из simulation.py.

Так поведенческий слой (mission.py) не знает, ездит он по-настоящему или в
симуляции: ему достаточно этих интерфейсов.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from .geometry import Pose, clamp


# --------------------------------------------------------------------------
#  Команда движения
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class DriveCommand:
    """Желаемое движение в нормированных единицах.

    linear  ∈ [-1, 1] — вперёд(+)/назад(-).
    angular ∈ [-1, 1] — поворот влево(+)/вправо(-) (против/по часовой).

    Конкретный Drive переводит это в проценты левого/правого мотора
    (дифференциальная схема) — см. to_differential().
    """

    linear: float = 0.0
    angular: float = 0.0

    def clamped(self) -> "DriveCommand":
        return DriveCommand(clamp(self.linear, -1.0, 1.0), clamp(self.angular, -1.0, 1.0))

    def to_differential(self) -> tuple[float, float]:
        """(left, right) в [-1, 1] для дифференциального привода."""
        c = self.clamped()
        left = clamp(c.linear - c.angular, -1.0, 1.0)
        right = clamp(c.linear + c.angular, -1.0, 1.0)
        return left, right

    @property
    def is_stop(self) -> bool:
        return self.linear == 0.0 and self.angular == 0.0


STOP = DriveCommand(0.0, 0.0)


# --------------------------------------------------------------------------
#  Что видит луноход: цель и препятствия
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class TargetView:
    """Свёрнутое представление цели для поведенческого слоя.

    Не зависит от того, YOLO это или симуляция. Строится из детекций
    (см. perception.build_target_view).
    """

    visible: bool = False
    offset: float = 0.0       # [-1..1], 0 = по центру кадра
    closeness: float = 0.0    # доля площади кадра, прокси близости
    bearing_deg: float = 0.0  # угол на цель относительно курса
    confidence: float = 0.0

    @staticmethod
    def none() -> "TargetView":
        return TargetView(visible=False)


@dataclass(frozen=True)
class RangeView:
    """Дальности до ближайших препятствий по трём секторам, метры.

    Большое значение = свободно. Источник — УЗ/ToF дальномеры (через UART)
    или, при желании, оценка по кадру. None = датчик недоступен.
    """

    left: float = float("inf")
    front: float = float("inf")
    right: float = float("inf")

    def clearest_side(self) -> str:
        """'left' или 'right' — куда свободнее повернуть."""
        return "left" if self.left >= self.right else "right"

    def front_blocked(self, stop_distance: float) -> bool:
        return self.front < stop_distance


class ManipState(Enum):
    STOWED = "stowed"     # поднят, пусто
    LOWERED = "lowered"   # опущен к образцу
    HOLDING = "holding"   # держит образец (поднят)


# --------------------------------------------------------------------------
#  Интерфейсы исполнительных устройств и датчиков
# --------------------------------------------------------------------------
class Drive(ABC):
    """Привод (два мотора / гусеницы)."""

    @abstractmethod
    def set_velocity(self, command: DriveCommand) -> None:
        ...

    def stop(self) -> None:
        self.set_velocity(STOP)

    def odometry(self) -> Optional[Pose]:
        """Поза по энкодерам, если есть. Иначе None — счислять по командам."""
        return None


class Manipulator(ABC):
    """Манипулятор/захват для сбора образца. Операции могут длиться
    несколько тиков; статус возвращает state()."""

    @abstractmethod
    def lower(self) -> None: ...

    @abstractmethod
    def grip(self) -> None: ...

    @abstractmethod
    def lift(self) -> None: ...

    @abstractmethod
    def release(self) -> None: ...

    @abstractmethod
    def state(self) -> ManipState: ...

    def is_holding(self) -> bool:
        return self.state() == ManipState.HOLDING


class RangeSensor(ABC):
    """Дальномеры препятствий."""

    @abstractmethod
    def read(self) -> RangeView: ...


class BoundarySensor(ABC):
    """Датчик границы (тёмная кромка под луноходом). Совместим по смыслу с
    boundary_level() из track.py: возвращает долю «черноты» 0..1."""

    @abstractmethod
    def level(self) -> float: ...


class TargetSensor(ABC):
    """Источник цели: для железа — камера+YOLO+navigation, для отладки — симуляция."""

    @abstractmethod
    def sense(self) -> TargetView: ...
