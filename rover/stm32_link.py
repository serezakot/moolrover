"""
stm32_link.py — UART-мост между Raspberry Pi 5 (Python) и STM32.

Архитектура (по аналогии с navigation.py / state_machine.py):
    - Чистые функции упаковки/разборки пакетов НЕ трогают железо и НЕ трогают
      потоки. Их можно тестировать на синтетических байтах без Pi и без STM32.
    - Класс STM32Link прячет внутри serial и два фоновых потока:
        * TX-поток: с фиксированной частотой (~50 Гц) шлёт ПОСЛЕДНЮЮ выставленную
          команду движения, даже если "мозг" молчит. Так watchdog STM32 (300 мс)
          всегда сыт, а логика управления развязана с реальным временем шины.
        * RX-поток: непрерывно ищет кадр телеметрии, проверяет CRC, распаковывает
          RLE и кладёт последнюю валидную телеметрию в потокобезопасное поле.

Как это стыкуется с RoverBrain:
    link = STM32Link(); link.start()
    ...
    # внутри brain.update() / track.py:
    link.set_speed(30, 30)          # мгновенно, не блокирует
    t = link.get_telemetry()        # последняя валидная телеметрия или None

ЗАФИКСИРОВАННЫЕ ПАРАМЕТРЫ ПРОТОКОЛА (согласовано с прошивкой STM32):
    - Скорость UART: 115200, 8N1.
    - Скорости моторов в телеметрии приходят ЗНАКОВЫМИ int8 (см. TELEM_SPEED_SIGNED).
      ^ единственное допущение, которое стоит перепроверить на живом железе:
        если задний ход придёт как модуль или направление окажется в статус-байте,
        меняется одна константа/одна ветка parse_telemetry_payload().
"""

import sys
import threading
import time
from collections import deque
from dataclasses import dataclass

try:
    import serial  # pyserial; не нужен для чистых функций и селф-теста
except ImportError:
    serial = None


# ─────────────────────────── Константы протокола ───────────────────────────

PORT = "/dev/serial0"     # на Pi 5: GPIO14 (TX) / GPIO15 (RX); включить в raspi-config
BAUD = 115200

# TX (команды на STM32) — пакет фиксированной длины 7 байт
TX_START = 0xAA
TX_STOP = 0xBB

CMD_ESTOP = 0x00          # аргументы 0,0,0
CMD_SET_SPEED = 0x01      # arg1=скорость L (int8), arg2=скорость R (int8), arg3=0
CMD_RESET_ODO = 0x02      # аргументы 0,0,0
CMD_TUNE_PID = 0x03       # arg1=коэф(1=Kp,2=Ki,3=Kd), arg2=целая часть, arg3=дробь*100
CMD_SERVO = 0x05          # arg1=id(0..3), arg2=угол(0..180), arg3=0

# RX (телеметрия от STM32) — старт/стоп маркеры
RX_START = 0xCC
RX_STOP = 0xDD

# Тайминги
TX_PERIOD = 0.02          # 50 Гц. Watchdog STM32 = 300 мс, запас огромный
RX_READ_TIMEOUT = 0.05    # таймаут одиночного serial.read

# Ограничения/размеры
TELEM_SIZE = 10           # распакованная телеметрия строго 10 байт
MAX_RLE_LEN = 32          # защита от мусора: N сжатых байт не может быть больше
TELEM_SPEED_SIGNED = True # см. шапку файла: скорости в телеметрии как int8


# ─────────────────────── Чистые функции: TX (команды) ───────────────────────

def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _pack_int8(v: int) -> int:
    """Знаковую скорость (-100..100) -> байт two's-complement (0..255)."""
    v = _clamp(int(v), -100, 100)
    return v & 0xFF


def _unpack_int8(b: int) -> int:
    """Байт -> знаковый int8 (-128..127)."""
    return b - 256 if b >= 128 else b


def build_command(cmd_id: int, a1: int = 0, a2: int = 0, a3: int = 0) -> bytes:
    """
    Собрать 7-байтный пакет команды.
    a1..a3 — УЖЕ упакованные байты (0..255). CRC = XOR байтов [cmd_id, a1, a2, a3].
    """
    cmd_id &= 0xFF
    a1 &= 0xFF
    a2 &= 0xFF
    a3 &= 0xFF
    crc = cmd_id ^ a1 ^ a2 ^ a3
    return bytes([TX_START, cmd_id, a1, a2, a3, crc, TX_STOP])


def cmd_set_speed(left: int, right: int) -> bytes:
    return build_command(CMD_SET_SPEED, _pack_int8(left), _pack_int8(right), 0)


def cmd_estop() -> bytes:
    return build_command(CMD_ESTOP, 0, 0, 0)


def cmd_reset_odometer() -> bytes:
    return build_command(CMD_RESET_ODO, 0, 0, 0)


def cmd_servo(servo_id: int, angle: int) -> bytes:
    servo_id = _clamp(int(servo_id), 0, 3)
    angle = _clamp(int(angle), 0, 180)
    return build_command(CMD_SERVO, servo_id, angle, 0)


def cmd_tune_pid(coef: int, value: float) -> bytes:
    """coef: 1=Kp, 2=Ki, 3=Kd. value -> целая часть + дробная*100."""
    ipart = int(value)
    fpart = int(round((value - ipart) * 100))
    return build_command(CMD_TUNE_PID, coef & 0xFF, ipart & 0xFF, fpart & 0xFF)


# ─────────────────────── Чистые функции: RX (телеметрия) ─────────────────────

@dataclass(frozen=True)
class Telemetry:
    status: int         # статусный байт робота
    battery_v: float    # напряжение, В (сырой байт / 10, т.е. 114 -> 11.4)
    speed_left: int     # текущая скорость L
    speed_right: int    # текущая скорость R
    sonar_cm: int       # расстояние с сонара, см
    raw: bytes          # 10 распакованных байт (для отладки)


def rx_crc(length: int, rle_bytes: bytes) -> int:
    """CRC телеметрии = length XOR всех сжатых байт."""
    c = length & 0xFF
    for x in rle_bytes:
        c ^= x
    return c & 0xFF


def rle_decompress(data: bytes) -> bytes:
    """Пары [значение, количество] -> развёрнутый массив байт."""
    out = bytearray()
    for i in range(0, len(data), 2):
        value = data[i]
        count = data[i + 1]
        out.extend([value] * count)
    return bytes(out)


def parse_telemetry_payload(payload: bytes) -> Telemetry:
    """Распакованные 10 байт -> Telemetry."""
    b = payload
    sl = _unpack_int8(b[2]) if TELEM_SPEED_SIGNED else b[2]
    sr = _unpack_int8(b[3]) if TELEM_SPEED_SIGNED else b[3]
    return Telemetry(
        status=b[0],
        battery_v=b[1] / 10.0,
        speed_left=sl,
        speed_right=sr,
        sonar_cm=b[4],
        raw=bytes(b),
    )


# ──────────────────────────── Класс-мост ─────────────────────────────────────

class STM32Link:
    def __init__(self, port: str = PORT, baud: int = BAUD):
        if serial is None:
            raise RuntimeError("Нужен pyserial: pip install pyserial")
        self._ser = serial.Serial(port, baud, timeout=RX_READ_TIMEOUT)

        # текущая команда движения (heartbeat), по умолчанию — безопасный стоп
        self._motion = cmd_estop()
        self._motion_lock = threading.Lock()

        # очередь разовых команд (серва / PID / сброс одометра)
        self._oneshot = deque()
        self._oneshot_lock = threading.Lock()

        # последняя валидная телеметрия
        self._telem = None
        self._telem_time = 0.0
        self._telem_lock = threading.Lock()

        self._running = False
        self._tx_thread = None
        self._rx_thread = None

    # ── управление жизненным циклом ──
    def start(self):
        if self._running:
            return
        self._running = True
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._tx_thread.start()
        self._rx_thread.start()

    def stop(self):
        self._running = False
        try:
            self._ser.write(cmd_estop())   # финальный стоп на всякий случай
        except Exception:
            pass
        if self._tx_thread:
            self._tx_thread.join(timeout=0.5)
        if self._rx_thread:
            self._rx_thread.join(timeout=0.5)
        try:
            self._ser.close()
        except Exception:
            pass

    # ── публичный API для мозга ──
    def set_speed(self, left: int, right: int):
        with self._motion_lock:
            self._motion = cmd_set_speed(left, right)

    def emergency_stop(self):
        # стоп важнее очереди — чистим разовые команды, чтобы не задержать его
        with self._oneshot_lock:
            self._oneshot.clear()
        with self._motion_lock:
            self._motion = cmd_estop()

    def set_servo(self, servo_id: int, angle: int):
        with self._oneshot_lock:
            self._oneshot.append(cmd_servo(servo_id, angle))

    def reset_odometer(self):
        with self._oneshot_lock:
            self._oneshot.append(cmd_reset_odometer())

    def tune_pid(self, coef: int, value: float):
        with self._oneshot_lock:
            self._oneshot.append(cmd_tune_pid(coef, value))

    def get_telemetry(self):
        with self._telem_lock:
            return self._telem

    def telemetry_age(self):
        """Секунд с последнего валидного кадра, или None если кадров ещё не было."""
        with self._telem_lock:
            if self._telem is None:
                return None
            return time.time() - self._telem_time

    # ── фоновые потоки ──
    def _tx_loop(self):
        next_t = time.time()
        while self._running:
            pkt = None
            with self._oneshot_lock:
                if self._oneshot:
                    pkt = self._oneshot.popleft()
            if pkt is None:
                with self._motion_lock:
                    pkt = self._motion
            try:
                self._ser.write(pkt)
            except Exception:
                pass  # не роняем поток из-за одной сбойной записи

            next_t += TX_PERIOD
            sleep = next_t - time.time()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.time()  # отстали — не копим долг

    def _rx_loop(self):
        while self._running:
            try:
                frame = self._read_frame()
            except Exception:
                frame = None
            if frame is None:
                continue
            with self._telem_lock:
                self._telem = frame
                self._telem_time = time.time()

    def _read_exact(self, n: int, deadline_s: float = 0.2):
        buf = bytearray()
        deadline = time.time() + deadline_s
        while len(buf) < n and time.time() < deadline:
            chunk = self._ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
        return bytes(buf) if len(buf) == n else None

    def _read_frame(self):
        ser = self._ser
        # 1. синхронизация по стартовому байту
        b = ser.read(1)
        if not b or b[0] != RX_START:
            return None
        # 2. длина сжатых данных
        lb = ser.read(1)
        if not lb:
            return None
        length = lb[0]
        if length == 0 or length % 2 != 0 or length > MAX_RLE_LEN:
            return None  # мусор -> дропаем, ищем следующий 0xCC
        # 3. данные + CRC + стоп
        rest = self._read_exact(length + 2)
        if rest is None:
            return None
        rle_bytes = rest[:length]
        crc = rest[length]
        stop = rest[length + 1]
        if stop != RX_STOP:
            return None
        if rx_crc(length, rle_bytes) != crc:
            return None
        payload = rle_decompress(rle_bytes)
        if len(payload) != TELEM_SIZE:
            return None
        return parse_telemetry_payload(payload)


# ─────────────────────────── Селф-тест (без железа) ──────────────────────────

def _rle_compress_naive(data: bytes) -> bytes:
    """Только для тестов: собрать кадр так, как это делает STM32."""
    out = bytearray()
    i = 0
    while i < len(data):
        v = data[i]
        c = 1
        while i + c < len(data) and data[i + c] == v and c < 255:
            c += 1
        out.extend([v, c])
        i += c
    return bytes(out)


def _selftest():
    # --- TX: упаковка знака и CRC ---
    pkt = cmd_set_speed(-50, 30)
    assert pkt[0] == TX_START and pkt[6] == TX_STOP
    assert pkt[1] == CMD_SET_SPEED
    assert pkt[2] == 206          # -50 two's-complement
    assert pkt[3] == 30
    assert pkt[5] == (pkt[1] ^ pkt[2] ^ pkt[3] ^ pkt[4])
    assert cmd_estop() == bytes([0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBB])
    assert cmd_servo(2, 90)[1:5] == bytes([0x05, 2, 90, 0])
    assert cmd_servo(9, 999)[3] == 180        # клампинг угла
    assert cmd_tune_pid(1, 1.25)[2:5] == bytes([1, 1, 25])

    # --- RX: сборка -> разбор кадра целиком ---
    payload = bytes([1, 114, 206, 30, 42, 0, 0, 0, 0, 0])  # -50, 30, 11.4V, 42см
    rle = _rle_compress_naive(payload)
    length = len(rle)
    crc = rx_crc(length, rle)
    frame = bytes([RX_START, length]) + rle + bytes([crc, RX_STOP])

    # проверим внутренности парсера на распакованном payload
    assert rle_decompress(rle) == payload
    t = parse_telemetry_payload(payload)
    assert t.status == 1
    assert abs(t.battery_v - 11.4) < 1e-9
    assert t.speed_left == -50 and t.speed_right == 30
    assert t.sonar_cm == 42

    # CRC-логика: битый байт должен ломать проверку
    assert rx_crc(length, bytes([rle[0] ^ 0xFF]) + rle[1:]) != crc

    print("OK: все чистые функции сходятся. Кадр телеметрии:",
          frame.hex(" "))


if __name__ == "__main__":
    if "--selftest" in sys.argv or serial is None:
        _selftest()
    else:
        # живой демо-режим (нужен реальный UART и STM32)
        link = STM32Link()
        link.start()
        try:
            link.set_speed(0, 0)
            while True:
                t = link.get_telemetry()
                if t:
                    print(f"bat={t.battery_v:.1f}V  L={t.speed_left}  "
                          f"R={t.speed_right}  sonar={t.sonar_cm}cm  "
                          f"status={t.status}  age={link.telemetry_age():.2f}s")
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        finally:
            link.stop()
