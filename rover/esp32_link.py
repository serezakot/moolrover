"""
esp32_link.py — UART-мост между Raspberry Pi 5 (Python) и ESP32-WROOM-32D.

Шаг 3: расширенная телеметрия (16 байт): heading, pos_x, pos_y, imu_status.

Архитектура:
    - Чистые функции упаковки/разборки пакетов НЕ трогают железо и НЕ трогают
      потоки. Их можно тестировать на синтетических байтах без Pi и без ESP32.
    - Класс ESP32Link прячет внутри serial и два фоновых потока:
        * TX-поток: 50 Гц heartbeat (watchdog ESP32 = 300 мс).
        * RX-поток: ищет кадр телеметрии, проверяет CRC, распаковывает RLE.

Как стыкуется с RoverBrain:
    link = ESP32Link(); link.start()
    link.set_speed(30, 30)
    t = link.get_telemetry()   # Telemetry | None
    if t:
        print(t.heading_deg, t.pos_x_cm, t.pos_y_cm)

ЗАФИКСИРОВАННЫЕ ПАРАМЕТРЫ ПРОТОКОЛА (согласовано с ESP32 step3):
    - UART: 115200, 8N1.
    - ESP32: Serial2 (GPIO16 RX ← Pi TX, GPIO17 TX → Pi RX).
    - Pi: /dev/ttyAMA0 (GPIO14 TX, GPIO15 RX).
    - Телеметрия: 16 байт (status, battery, speeds, sonar, heading, x, y, imu_status, reserved).
"""

import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass

try:
    import serial  # pyserial
except ImportError:
    serial = None


# ─────────────────────────── Константы протокола ───────────────────────────

PORT = "/dev/serial0"
BAUD = 115200

# TX (команды на ESP32)
TX_START = 0xAA
TX_STOP = 0xBB

CMD_ESTOP = 0x00
CMD_SET_SPEED = 0x01
CMD_RESET_ODO = 0x02
CMD_TUNE_PID = 0x03
CMD_SET_PERIPH = 0x04
CMD_SERVO = 0x05
CMD_RESET_IMU = 0x06      # НОВАЯ: сброс heading и координат

# RX (телеметрия от ESP32)
RX_START = 0xCC
RX_STOP = 0xDD

# Тайминги
TX_PERIOD = 0.02
RX_READ_TIMEOUT = 0.05

# Размеры
TELEM_SIZE = 16            # БЫЛО 10, СТАЛО 16 — согласовано с rover_types.h
MAX_RLE_LEN = 64           # увеличено под 16-байтную телеметрию
TELEM_SPEED_SIGNED = True


# ─────────────────────── Чистые функции: TX (команды) ───────────────────────

def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _pack_int8(v: int) -> int:
    v = _clamp(int(v), -100, 100)
    return v & 0xFF


def _unpack_int8(b: int) -> int:
    return b - 256 if b >= 128 else b


def build_command(cmd_id: int, a1: int = 0, a2: int = 0, a3: int = 0) -> bytes:
    cmd_id &= 0xFF; a1 &= 0xFF; a2 &= 0xFF; a3 &= 0xFF
    crc = cmd_id ^ a1 ^ a2 ^ a3
    return bytes([TX_START, cmd_id, a1, a2, a3, crc, TX_STOP])


def cmd_set_speed(left: int, right: int) -> bytes:
    return build_command(CMD_SET_SPEED, _pack_int8(left), _pack_int8(right), 0)


def cmd_estop() -> bytes:
    return build_command(CMD_ESTOP, 0, 0, 0)


def cmd_reset_odometer() -> bytes:
    return build_command(CMD_RESET_ODO, 0, 0, 0)


def cmd_servo(servo_id: int, angle: int) -> bytes:
    servo_id = _clamp(int(servo_id), 0, 1)
    angle = _clamp(int(angle), 0, 180)
    return build_command(CMD_SERVO, servo_id, angle, 0)


def cmd_tune_pid(coef: int, value: float) -> bytes:
    ipart = int(value)
    fpart = int(round((value - ipart) * 100))
    return build_command(CMD_TUNE_PID, coef & 0xFF, ipart & 0xFF, fpart & 0xFF)


def cmd_reset_imu() -> bytes:
    """Сбросить heading и координаты на ESP32 в 0."""
    return build_command(CMD_RESET_IMU, 0, 0, 0)


# ─────────────────────── Чистые функции: RX (телеметрия) ─────────────────────

@dataclass(frozen=True)
class Telemetry:
    status: int
    battery_v: float
    speed_left: int          # проценты, -100..100
    speed_right: int
    sonar_cm: int
    heading_deg: float       # НОВОЕ: курс от старта, градусы (-180..180)
    pos_x_cm: int            # НОВОЕ: x от старта, см
    pos_y_cm: int            # НОВОЕ: y от старта, см
    imu_status: int          # НОВОЕ: 0=нет, 1=ок, 2=ошибка
    raw: bytes


def rx_crc(length: int, rle_bytes: bytes) -> int:
    c = length & 0xFF
    for x in rle_bytes:
        c ^= x
    return c & 0xFF


def rle_decompress(data: bytes) -> bytes:
    out = bytearray()
    for i in range(0, len(data), 2):
        value = data[i]
        count = data[i + 1]
        out.extend([value] * count)
    return bytes(out)


def parse_telemetry_payload(payload: bytes) -> Telemetry:
    """Распакованные 16 байт -> Telemetry."""
    b = payload
    sl = _unpack_int8(b[2]) if TELEM_SPEED_SIGNED else b[2]
    sr = _unpack_int8(b[3]) if TELEM_SPEED_SIGNED else b[3]

    # heading_deg10: int16 little-endian в байтах 5-6
    heading_raw = struct.unpack_from('<h', b, 5)[0]
    heading_deg = heading_raw / 10.0

    # pos_x_cm, pos_y_cm: int16 LE в байтах 7-8, 9-10
    pos_x_cm = struct.unpack_from('<h', b, 7)[0]
    pos_y_cm = struct.unpack_from('<h', b, 9)[0]

    imu_status = b[11]

    return Telemetry(
        status=b[0],
        battery_v=b[1] / 10.0,
        speed_left=sl,
        speed_right=sr,
        sonar_cm=b[4],
        heading_deg=heading_deg,
        pos_x_cm=pos_x_cm,
        pos_y_cm=pos_y_cm,
        imu_status=imu_status,
        raw=bytes(b),
    )


# ──────────────────────────── Класс-мост ─────────────────────────────────────

class ESP32Link:
    def __init__(self, port: str = PORT, baud: int = BAUD):
        if serial is None:
            raise RuntimeError("Нужен pyserial: pip install pyserial")
        self._ser = serial.Serial(port, baud, timeout=RX_READ_TIMEOUT)

        self._motion = cmd_estop()
        self._motion_lock = threading.Lock()

        self._oneshot = deque()
        self._oneshot_lock = threading.Lock()

        self._telem = None
        self._telem_time = 0.0
        self._telem_lock = threading.Lock()

        self._running = False
        self._tx_thread = None
        self._rx_thread = None

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
            self._ser.write(cmd_estop())
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

    # ── публичный API ──
    def set_speed(self, left: int, right: int):
        with self._motion_lock:
            self._motion = cmd_set_speed(left, right)

    def emergency_stop(self):
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

    def reset_imu(self):
        """Обнулить heading и координаты на ESP32."""
        with self._oneshot_lock:
            self._oneshot.append(cmd_reset_imu())

    def tune_pid(self, coef: int, value: float):
        with self._oneshot_lock:
            self._oneshot.append(cmd_tune_pid(coef, value))

    def get_telemetry(self):
        with self._telem_lock:
            return self._telem

    def telemetry_age(self):
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
                pass
            next_t += TX_PERIOD
            sleep = next_t - time.time()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.time()

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
        b = ser.read(1)
        if not b or b[0] != RX_START:
            return None
        lb = ser.read(1)
        if not lb:
            return None
        length = lb[0]
        if length == 0 or length % 2 != 0 or length > MAX_RLE_LEN:
            return None
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
    # --- TX ---
    pkt = cmd_set_speed(-50, 30)
    assert pkt[0] == TX_START and pkt[6] == TX_STOP
    assert pkt[1] == CMD_SET_SPEED
    assert pkt[2] == 206
    assert pkt[3] == 30
    assert pkt[5] == (pkt[1] ^ pkt[2] ^ pkt[3] ^ pkt[4])
    assert cmd_estop() == bytes([0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBB])
    assert cmd_servo(0, 90)[1:5] == bytes([0x05, 0, 90, 0])
    assert cmd_reset_imu()[1] == CMD_RESET_IMU

    # --- RX: 16-байтная телеметрия ---
    #   heading_deg10 = 453 (45.3°), pos_x = 150 cm, pos_y = -80 cm, imu=1
    heading_bytes = struct.pack('<h', 453)
    x_bytes = struct.pack('<h', 150)
    y_bytes = struct.pack('<h', -80)
    payload = bytes([
        1,            # status
        114,          # battery (11.4V)
        206,          # left_speed = -50 (two's complement)
        30,           # right_speed = 30
        42,           # sonar = 42 cm
    ]) + heading_bytes + x_bytes + y_bytes + bytes([
        1,            # imu_status = ok
        0, 0, 0, 0,  # reserved
    ])
    assert len(payload) == TELEM_SIZE, f"payload len={len(payload)}, expected {TELEM_SIZE}"

    rle = _rle_compress_naive(payload)
    length = len(rle)
    crc = rx_crc(length, rle)
    frame = bytes([RX_START, length]) + rle + bytes([crc, RX_STOP])

    assert rle_decompress(rle) == payload
    t = parse_telemetry_payload(payload)
    assert t.status == 1
    assert abs(t.battery_v - 11.4) < 1e-9
    assert t.speed_left == -50 and t.speed_right == 30
    assert t.sonar_cm == 42
    assert abs(t.heading_deg - 45.3) < 0.01
    assert t.pos_x_cm == 150
    assert t.pos_y_cm == -80
    assert t.imu_status == 1

    # CRC: битый байт должен ломать
    assert rx_crc(length, bytes([rle[0] ^ 0xFF]) + rle[1:]) != crc

    print(f"OK: selftest passed. Telemetry size={TELEM_SIZE}, frame={frame.hex(' ')}")
    print(f"    heading={t.heading_deg}°  pos=({t.pos_x_cm},{t.pos_y_cm})cm  imu={t.imu_status}")


if __name__ == "__main__":
    if "--selftest" in sys.argv or serial is None:
        _selftest()
    else:
        link = ESP32Link()
        link.start()
        try:
            link.set_speed(0, 0)
            while True:
                t = link.get_telemetry()
                if t:
                    print(f"bat={t.battery_v:.1f}V  L={t.speed_left}  "
                          f"R={t.speed_right}  sonar={t.sonar_cm}cm  "
                          f"hdg={t.heading_deg:.1f}°  "
                          f"pos=({t.pos_x_cm},{t.pos_y_cm})cm  "
                          f"imu={t.imu_status}  "
                          f"age={link.telemetry_age():.2f}s")
                else:
                    print("NO TELEMETRY")
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        finally:
            link.stop()
