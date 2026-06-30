"""Тесты планировщика поиска: круговой обзор, доворот к последнему контакту,
объезд препятствия при переезде.

    python3 tests/test_search.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rover.geometry import Pose  # noqa: E402
from rover.hardware import RangeView  # noqa: E402
from rover.search import SearchPlanner  # noqa: E402


def _drive(planner, pose, ranges, ticks):
    """Прогнать планировщик ticks тактов, эмулируя одометрию по командам."""
    cmds = []
    for _ in range(ticks):
        cmd = planner.step(pose, ranges)
        pose.rotate(cmd.angular * 12.0)       # 12 град/тик
        pose.advance(cmd.linear * 0.15)
        cmds.append(cmd)
    return cmds


def _drive_until_mode(planner, pose, ranges, mode, max_ticks=200):
    """Шагать, пока планировщик не войдёт в режим mode (или лимит тактов)."""
    for _ in range(max_ticks):
        cmd = planner.step(pose, ranges)
        if planner.mode == mode:
            return True
        pose.rotate(cmd.angular * 12.0)
        pose.advance(cmd.linear * 0.15)
    return False


def test_scan_rotates_in_place():
    sp = SearchPlanner()
    sp.on_enter(None)
    cmd = sp.step(Pose(), RangeView())
    assert cmd.linear == 0.0 and cmd.angular != 0.0     # крутится, не едет


def test_scan_completes_to_relocate():
    sp = SearchPlanner()
    sp.on_enter(None)
    # после полного кругового обзора планировщик переходит к переезду
    assert _drive_until_mode(sp, Pose(), RangeView(), "RELOCATE")


def test_reacquire_turns_toward_last_seen():
    # цель в последний раз справа (bearing>0) -> доворот вправо (angular<0)
    sp = SearchPlanner()
    sp.on_enter(40.0)
    cmd = sp.step(Pose(), RangeView())
    assert sp.mode == "REACQUIRE"
    assert cmd.angular < 0

    sp2 = SearchPlanner()
    sp2.on_enter(-40.0)                                  # слева -> влево
    assert sp2.step(Pose(), RangeView()).angular > 0


def test_relocate_avoids_blocked_front():
    sp = SearchPlanner()
    sp.on_enter(None)
    pose = Pose()
    assert _drive_until_mode(sp, pose, RangeView(), "RELOCATE")
    # фронт занят, слева свободно -> поворот влево (angular>0), без хода вперёд
    blocked = RangeView(left=3.0, front=0.1, right=0.1)
    cmd = sp.step(pose, blocked)
    assert cmd.linear == 0.0 and cmd.angular > 0


def test_visited_memory_grows():
    sp = SearchPlanner()
    sp.on_enter(None)
    pose = Pose()
    _drive(sp, pose, RangeView(), 120)
    assert len(sp.visited) >= 1                          # запоминаем, где были


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"  ok  {fn.__name__}")
    print(f"\n{len(fns)} тестов пройдено")


if __name__ == "__main__":
    _run_all()
