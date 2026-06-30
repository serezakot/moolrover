"""Протокол UART к STM32 и реализация привода/манипулятора поверх него.

Сейчас прошивка STM32 (uart_version_1) — это loopback. Здесь задаётся
компактный кадровый протокол, который контроллеру предстоит разобрать, чтобы
крутить моторы и манипулятор. Питон-сторона (Raspberry Pi) уже готова.

Формат кадра (байты):

    0xAA  CMD  LEN  payload[LEN]  CRC8

    0xAA  — стартовый байт (синхронизация);
    CMD   — команда (см. ниже);
    LEN   — длина payload;
    CRC8  — контрольная сумма по CMD+LEN+payload (полином 0x07).

Команды:

    0x01 DRIVE  payload = int8 left, int8 right   (проценты -100..100)
    0x02 MANIP  payload = uint8 action            (0=release 1=grip 2=lower 3=lift)
    0x03 STOP   payload = (пусто)
    0x10 PING   payload = (пусто)

pyserial подгружается лениво — модуль импортируется и тестируется без него
(кодирование кадров чистое и проверяемо offline).
"""

from __future__ import annotations

from typing import Optional

from .hardware import Drive, DriveCommand, ManipState, Manipulator
from .geometry import clamp

START = 0xAA
CMD_DRIVE = 0x01
CMD_MANIP = 0x02
CMD_STOP = 0x03
CMD_PING = 0x10

ACT_RELEASE = 0
ACT_GRIP = 1
ACT_LOWER = 2
ACT_LIFT = 3


def crc8(data: bytes, poly: int = 0x07) -> int:
    """CRC-8 (как на STM32 при ручной реализации)."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def _frame(cmd: int, payload: bytes = b"") -> bytes:
    body = bytes([cmd, len(payload)]) + payload
    return bytes([START]) + body + bytes([crc8(body)])


def _i8(percent: float) -> int:
    """Нормированное [-1..1] -> знаковый байт -100..100."""
    v = int(round(clamp(percent, -1.0, 1.0) * 100))
    return v & 0xFF  # two's complement в один байт


def encode_drive(left: float, right: float) -> bytes:
    """Кадр DRIVE из нормированных скоростей колёс [-1..1]."""
    return _frame(CMD_DRIVE, bytes([_i8(left), _i8(right)]))


def encode_manip(action: int) -> bytes:
    return _frame(CMD_MANIP, bytes([action & 0xFF]))


def encode_stop() -> bytes:
    return _frame(CMD_STOP)


def encode_ping() -> bytes:
    return _frame(CMD_PING)


class Stm32Link(Drive, Manipulator):
    """Привод + манипулятор через UART к STM32.

    Реализует оба интерфейса: один физический канал, разные команды.
    pyserial открывается лениво при первом использовании, чтобы модуль
    оставался импортируемым на машине без железа.
    """

    def __init__(self, port: str = "/dev/serial0", baudrate: int = 9600):
        self.port = port
        self.baudrate = baudrate
        self._serial = None
        self._manip = ManipState.STOWED

    def _io(self):
        if self._serial is None:
            try:
                import serial  # pyserial
            except ImportError as exc:  # pragma: no cover
                raise ImportError(
                    "Stm32Link требует pyserial (pip install pyserial) на Raspberry Pi."
                ) from exc
            self._serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
        return self._serial

    def _send(self, frame: bytes) -> None:
        self._io().write(frame)

    # --- Drive ---
    def set_velocity(self, command: DriveCommand) -> None:
        left, right = command.clamped().to_differential()
        self._send(encode_drive(left, right))

    def stop(self) -> None:
        self._send(encode_stop())

    # --- Manipulator ---
    def lower(self) -> None:
        self._send(encode_manip(ACT_LOWER))
        self._manip = ManipState.LOWERED

    def grip(self) -> None:
        self._send(encode_manip(ACT_GRIP))
        self._manip = ManipState.HOLDING

    def lift(self) -> None:
        self._send(encode_manip(ACT_LIFT))

    def release(self) -> None:
        self._send(encode_manip(ACT_RELEASE))
        self._manip = ManipState.STOWED

    def state(self) -> ManipState:
        return self._manip

    def odometry(self):  # энкодеры пока не заведены
        return None
