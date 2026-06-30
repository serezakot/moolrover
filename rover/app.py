"""Точка входа: собрать миссию из реализаций интерфейсов и гонять её.

Два режима:
    sim  — прогон логики в симуляции (по умолчанию, без железа);
    real — на Raspberry Pi: камера+YOLO для цели и STM32 по UART для моторов
           и манипулятора. Каркас real-режима показан, но требует picamera2/
           ultralytics/pyserial и собранной прошивки STM32.

Запуск симуляции:
    python3 -m rover.app
"""

from __future__ import annotations

import sys

from .mission import RoverMission, State
from .simulation import (
    SimBoundarySensor,
    SimDrive,
    SimManipulator,
    SimRangeSensor,
    SimTargetSensor,
    build_demo_world,
)


def make_sim_mission():
    world = build_demo_world()
    drive = SimDrive(world)
    manip = SimManipulator(world)
    mission = RoverMission(
        drive=drive,
        manipulator=manip,
        target_sensor=SimTargetSensor(world),
        range_sensor=SimRangeSensor(world),
        boundary_sensor=SimBoundarySensor(world),
    )
    return mission, world


def run_sim(max_ticks: int = 1500, verbose: bool = True) -> int:
    mission, world = make_sim_mission()
    total = len(world.targets)
    last_state = None
    last_note = None

    for tick in range(max_ticks):
        result = mission.step()
        if verbose and (result.state != last_state or result.note != last_note):
            t = result.target
            seen = f"target conf={t.confidence:.2f} off={t.offset:+.2f} near={t.closeness:.2f}" \
                if t.visible else "target: —"
            print(f"[{tick:4d}] {result.state.value:8s} {result.note:18s} "
                  f"| {seen} | collected={result.collected}")
            last_state, last_note = result.state, result.note
        if result.collected >= total:
            print(f"\nВсе образцы собраны за {tick} тактов.")
            break

    print(f"Итог: собрано {mission.collected} из {total}.")
    return mission.collected


def make_real_mission():  # pragma: no cover - требует железа
    """Каркас под реальный луноход. Камера+YOLO дают детекции, которые
    perception.build_target_view превращает в TargetView; STM32 по UART —
    привод и манипулятор."""
    raise NotImplementedError(
        "Real-режим: подставьте YoloTargetSensor (camera+ultralytics+perception) "
        "и rover.uart_link.Stm32Link как Drive+Manipulator в RoverMission. "
        "Дальномеры/датчик границы — с GPIO/UART или из кадра (см. track.boundary_level)."
    )


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "sim"
    if mode == "real":
        make_real_mission()
    else:
        collected = run_sim()
        sys.exit(0 if collected > 0 else 1)
