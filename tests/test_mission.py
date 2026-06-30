"""Тесты конечного автомата миссии на симуляции: полный цикл сбора,
приоритет границы, центрирование при подъезде, порог уверенности.

    python3 tests/test_mission.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rover.app import make_sim_mission  # noqa: E402
from rover.geometry import Pose  # noqa: E402
from rover.mission import RoverMission, State  # noqa: E402
from rover.simulation import (  # noqa: E402
    SimBoundarySensor,
    SimConfig,
    SimDrive,
    SimManipulator,
    SimRangeSensor,
    SimTargetSensor,
    SimWorld,
)
from rover.perception import build_target_view  # noqa: E402


def test_full_mission_collects_all_samples():
    mission, world = make_sim_mission()
    total = len(world.targets)
    for _ in range(1500):
        mission.step()
        if mission.collected >= total:
            break
    assert mission.collected == total, f"собрано {mission.collected}/{total}"


def test_boundary_triggers_avoid():
    world = SimWorld(config=SimConfig())
    world.rover = Pose(4.7, 0.0, 0.0)            # у самого края, носом наружу
    mission = RoverMission(
        SimDrive(world), SimManipulator(world),
        SimTargetSensor(world), SimRangeSensor(world), SimBoundarySensor(world),
    )
    result = mission.step()
    assert result.boundary >= mission.cfg.boundary_level
    assert result.state == State.AVOID


def test_approach_centers_target():
    # цель впереди-слева в поле зрения: подъезд должен уменьшать |offset|
    world = SimWorld(config=SimConfig())
    world.rover = Pose(0.0, 0.0, 0.0)
    world.targets = [(2.2, 0.5)]                 # различимо и нецентрировано
    mission = RoverMission(
        SimDrive(world), SimManipulator(world),
        SimTargetSensor(world), SimRangeSensor(world), SimBoundarySensor(world),
    )
    first = mission.step().target
    last = first
    for _ in range(6):
        last = mission.step().target
    assert first.visible and last.visible
    assert abs(last.offset) < abs(first.offset)


def test_low_confidence_target_is_invisible():
    # очень далёкая цель -> низкая уверенность -> отбрасывается порогом
    world = SimWorld(config=SimConfig())
    world.rover = Pose(0.0, 0.0, 0.0)
    world.targets = [(5.5, 0.0)]                 # далеко, conf < порога
    view = SimTargetSensor(world).sense()
    assert not view.visible


def test_perception_bridge_applies_threshold():
    # детекция с низкой уверенностью не превращается в видимую цель
    low = [(100, 100, 160, 160, 0.2)]
    assert not build_target_view(low, 1280, 720).visible
    high = [(100, 100, 160, 160, 0.9)]
    assert build_target_view(high, 1280, 720).visible


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"  ok  {fn.__name__}")
    print(f"\n{len(fns)} тестов пройдено")


if __name__ == "__main__":
    _run_all()
