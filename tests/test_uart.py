"""Тесты протокола UART к STM32 и дифференциального привода.

Чистый Python (pyserial не нужен — проверяем только кодирование кадров):

    python3 tests/test_uart.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rover.hardware import DriveCommand  # noqa: E402
from rover.uart_link import (  # noqa: E402
    CMD_DRIVE,
    CMD_STOP,
    START,
    crc8,
    encode_drive,
    encode_stop,
)


def test_frame_structure_and_crc():
    frame = encode_drive(0.5, -0.5)
    assert frame[0] == START
    assert frame[1] == CMD_DRIVE
    assert frame[2] == 2                      # длина payload
    assert frame[3] == 50                     # +0.5 -> +50%
    assert frame[4] == (256 - 50)             # -0.5 -> -50 (доп. код)
    assert frame[-1] == crc8(frame[1:-1])     # контрольная сумма по телу


def test_stop_frame():
    frame = encode_stop()
    assert frame[0] == START and frame[1] == CMD_STOP and frame[2] == 0
    assert frame[-1] == crc8(frame[1:-1])


def test_drive_clamped_to_byte_range():
    frame = encode_drive(5.0, -5.0)           # за пределами [-1,1]
    assert frame[3] == 100                     # ограничено +100%
    assert frame[4] == (256 - 100)             # и -100%


def test_differential_mixing():
    # вперёд прямо -> оба колеса одинаково
    assert DriveCommand(1.0, 0.0).to_differential() == (1.0, 1.0)
    # поворот влево (angular>0) -> левое медленнее правого
    left, right = DriveCommand(0.0, 1.0).to_differential()
    assert left < right
    # назад -> оба отрицательные
    assert DriveCommand(-1.0, 0.0).to_differential() == (-1.0, -1.0)


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"  ok  {fn.__name__}")
    print(f"\n{len(fns)} тестов пройдено")


if __name__ == "__main__":
    _run_all()
