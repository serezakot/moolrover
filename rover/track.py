"""
track.py — главный процесс лунохода на Raspberry Pi 5.

Фоновый поток управления (control_loop) по кругу делает ВСЁ:
    основная камера → YOLO → вторая камера (граница + жёлтая яма)
    → мозг (state_machine) → моторы (STM32)
Flask только ПОКАЗЫВАЕТ последние кадры, которые подготовил этот поток.
Поэтому луноход работает независимо от того, открыта ли страница в браузере
и сколько вкладок открыто.

Запуск:  cd rover && python3 track.py   →   http://<ip-малинки>:5000/
"""

import threading
import time
import traceback
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, Response
from picamera2 import Picamera2
from ultralytics import YOLO

# автонастройка камеры под освещение + проверка синего по HSV
from camera_setup import auto_calibrate, blue_fraction
# команда мозга → скорости моторов
from motor_controller import command_to_speed
# сенсорные функции ("глаза")
from navigation import target_angle, target_distance
# автомат ("мозг" с памятью)
from state_machine import RoverBrain
# UART-мост к STM32
from stm32_link import STM32Link

# --- Фильтр "реально ли синий" -------------------------------------------
USE_BLUE_FILTER = True     # отсекать боксы YOLO, в которых мало синего
BLUE_MIN_FRACTION = 0.20   # минимальная доля синих пикселей в боксе (0..1)

# --- Вторая (нижняя) камера: граница поля + жёлтая яма --------------------
# Номер /dev/videoN. На Pi 5 CSI-камера сама занимает несколько /dev/video*,
# поэтому проверь реальный номер USB-камеры: v4l2-ctl --list-devices
BOUNDARY_CAM_INDEX = 0

# --- Распознавание границы: яркое (ретрорефлектор) ИЛИ резкий контраст ---
BAND_HEIGHT = 0.30         # какую часть кадра снизу считаем "землёй перед луноходом"
BRIGHT_THRESHOLD = 5       # ярче медианы пола в N раз = ретрорефлектор
CONTRAST_KERNEL = 15       # размер окна для поиска резкого перепада яркости
CONTRAST_THRESHOLD = 250   # минимальный перепад яркости = "граница, а не тень"
ZONE_TRIGGER = 0.05        # доля пикселей границы в зоне, чтобы зона "сработала"
ABS_BRIGHT = 150           # абсолютный минимум яркости для ретрорефлектора

# --- Детекция жёлтой ямы (deposit pit) по HSV ---
YELLOW_LOW = np.array([15, 80, 80])
YELLOW_HIGH = np.array([35, 255, 255])
YELLOW_TRIGGER = 0.05      # доля жёлтых пикселей в кадре, чтобы считать "я у ямы"


# ─────────────────────── Чистые функции зрения ───────────────────────────────

def boundary_zone(frame_bgr):
    """
    Ищет границу поля двумя способами:
    1. Яркие пиксели (ретрорефлектор) — основной сигнал.
    2. Резкий перепад яркости (контраст) — страховка.
    Тени дают плавное затемнение, низкий контраст → не тригерят.
    Граница поля = резкий скачок белое↔чёрное → высокий контраст → тригерит.

    Возвращает (сторона, (доля_L, доля_C, доля_R)), сторона — 'LEFT' / 'CENTER' /
    'RIGHT' или None, если границы нет.
    """
    h, w = frame_bgr.shape[:2]
    band = frame_bgr[int(h * (1 - BAND_HEIGHT)):, :]
    gray = cv2.cvtColor(band, cv2.COLOR_BGR2GRAY)

    floor = float(np.median(gray))

    # 1. Яркое — ретрорефлектор
    bright = (gray > floor * BRIGHT_THRESHOLD) & (gray > ABS_BRIGHT)

    # 2. Контраст — резкий перепад яркости (граница, не тень)
    kernel = np.ones((CONTRAST_KERNEL, CONTRAST_KERNEL), np.uint8)
    local_max = cv2.dilate(gray, kernel)
    local_min = cv2.erode(gray, kernel)
    high_contrast = (local_max.astype(np.int16) - local_min) > CONTRAST_THRESHOLD

    # комбинация: нашли яркое ИЛИ резкий контраст
    boundary = bright | high_contrast

    third = w // 3
    left_frac = float(boundary[:, :third].mean())
    center_frac = float(boundary[:, third:2 * third].mean())
    right_frac = float(boundary[:, 2 * third:].mean())
    fracs = (left_frac, center_frac, right_frac)

    if center_frac >= ZONE_TRIGGER:
        return "CENTER", fracs           # прямо перед носом — самое важное
    if max(left_frac, right_frac) < ZONE_TRIGGER:
        return None, fracs               # чисто
    return ("LEFT" if left_frac > right_frac else "RIGHT"), fracs


def detect_deposit(frame_bgr):
    """
    Ищет жёлтый цвет (яма) в кадре второй камеры.
    Возвращает (True, fraction) если жёлтого достаточно, иначе (False, fraction).
    """
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, YELLOW_LOW, YELLOW_HIGH)
    fraction = float(mask.mean()) / 255.0
    return fraction >= YELLOW_TRIGGER, fraction


def draw_boundary_overlay(frame2, b_side, b_fracs, yellow_frac):
    """Рисует на кадре второй камеры зоны границы и жёлтый детект (для стрима)."""
    h, w = frame2.shape[:2]
    band_top = int(h * (1 - BAND_HEIGHT))
    third = w // 3

    # полупрозрачные прямоугольники по зонам: зелёный = чисто, красный = граница
    overlay = frame2.copy()
    for i, frac in enumerate(b_fracs):
        color = (0, 0, 255) if frac >= ZONE_TRIGGER else (0, 255, 0)
        x_end = w if i == 2 else (i + 1) * third
        cv2.rectangle(overlay, (i * third, band_top), (x_end, h), color, -1)
    cv2.addWeighted(overlay, 0.3, frame2, 0.7, 0, frame2)

    # белые линии-разделители зон
    cv2.line(frame2, (third, band_top), (third, h), (255, 255, 255), 1)
    cv2.line(frame2, (2 * third, band_top), (2 * third, h), (255, 255, 255), 1)
    cv2.line(frame2, (0, band_top), (w, band_top), (255, 255, 255), 1)

    # подписи зон с долей пикселей границы
    for i, (lbl, frac) in enumerate(zip("LCR", b_fracs)):
        cx = i * third + third // 2
        cv2.putText(frame2, f"{lbl}:{frac:.2f}", (cx - 30, band_top + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # общий вердикт
    verdict = b_side if b_side else "CLEAR"
    color = (0, 0, 255) if b_side else (0, 255, 0)
    cv2.putText(frame2, f"BOUNDARY: {verdict}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    # жёлтый детект
    yellow_color = (0, 255, 255) if yellow_frac >= YELLOW_TRIGGER else (200, 200, 200)
    cv2.putText(frame2, f"YELLOW: {yellow_frac:.2f}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, yellow_color, 2)


# ─────────────────────── Железо (инициализация при запуске) ──────────────────

app = Flask(__name__)

# Основная камера
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (1280, 720)})
picam2.configure(config)
picam2.start()

# Автокалибровка баланса белого / экспозиции под ТЕКУЩИЙ свет.
# Наведи камеру на нейтральный фон — белый/серый лист — на момент старта.
auto_calibrate(picam2, settle_seconds=2.0)

# Вторая камера
boundary_cam = cv2.VideoCapture(BOUNDARY_CAM_INDEX)
if not boundary_cam.isOpened():
    raise RuntimeError(
        f"Не удалось открыть USB-камеру /dev/video{BOUNDARY_CAM_INDEX}. "
        f"Проверь: v4l2-ctl --list-devices"
    )

# Связь с моторами через STM32
link = STM32Link()
link.start()
print("[motors] STM32 link started")

# YOLO модель
PROJECT_ROOT = Path(__file__).resolve().parent
MODEL_CANDIDATES = [PROJECT_ROOT / "best.pt", PROJECT_ROOT / "yolo11n.pt"]
MODEL_PATH = next((p for p in MODEL_CANDIDATES if p.exists()), None)
if MODEL_PATH is None:
    raise FileNotFoundError(
        f"YOLO weights not found. Looked for: {[str(p) for p in MODEL_CANDIDATES]}"
    )
model = YOLO(str(MODEL_PATH))

# Мозг лунохода. Доступ из двух потоков (управление + /restart) — под замком.
brain = RoverBrain()
brain_lock = threading.Lock()


# ─────────────────────── Цикл управления ─────────────────────────────────────

# Последние готовые кадры для стрима (пишет поток управления, читает Flask)
_frames = {"video": None, "boundary": None}
_frame_seq = 0
_frames_cond = threading.Condition()
_stop_event = threading.Event()


def control_step():
    """Один такт: кадры → детекции → мозг → моторы. Возвращает кадры для стрима."""
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

    # YOLO tracking
    results = model.track(frame, persist=True, tracker="bytetrack.yaml", verbose=False)
    annotated = results[0].plot()

    # боксы + уверенность YOLO; каждый бокс проверяем по доле синего
    detections = []
    best_blue = 0.0
    boxes = results[0].boxes
    if boxes is not None and len(boxes):
        for (x1, y1, x2, y2), conf in zip(boxes.xyxy.tolist(), boxes.conf.tolist()):
            bf = blue_fraction(frame, (x1, y1, x2, y2))
            best_blue = max(best_blue, bf)
            if USE_BLUE_FILTER and bf < BLUE_MIN_FRACTION:
                continue                 # мало синего -> не наш образец
            # conf идёт дальше: navigation отбросит слабые (< CONFIDENCE_THRESHOLD)
            detections.append((x1, y1, x2, y2, conf))

    h, w = annotated.shape[:2]

    # кадр со второй камеры (граница + яма)
    ok, boundary_frame = boundary_cam.read()
    if ok:
        boundary_frame = cv2.flip(boundary_frame, -1)   # поворот на 180°
        b_side, b_fracs = boundary_zone(boundary_frame)
        at_deposit, yellow_frac = detect_deposit(boundary_frame)
        draw_boundary_overlay(boundary_frame, b_side, b_fracs, yellow_frac)
        with brain_lock:
            command = brain.update(detections, w, h, boundary_side=b_side,
                                   heading=None, at_deposit=at_deposit)
    else:
        # Без нижней камеры не видно границу поля — ехать нельзя.
        boundary_frame = None
        b_side, b_fracs = "NO CAMERA", (0.0, 0.0, 0.0)
        command = "STOP"

    # отправляем команду на моторы
    left_speed, right_speed = command_to_speed(command)
    link.set_speed(left_speed, right_speed)

    # --- отладочные надписи на основном кадре ---
    angle = target_angle(detections, w)
    dist = target_distance(detections, w)

    label = f"{brain.state} | {command}"
    if angle is not None:
        label += f"  ({angle:+.0f} deg)"
    if dist is not None:
        label += f"  {dist:.2f}m"
    cv2.putText(annotated, label, (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
    cv2.putText(annotated,
                f"border: {b_side} L{b_fracs[0]:.2f} C{b_fracs[1]:.2f} R{b_fracs[2]:.2f}",
                (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(annotated, f"blue: {best_blue:.2f} / min {BLUE_MIN_FRACTION:.2f}",
                (20, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
    mins, secs = divmod(int(brain.time_left()), 60)
    cv2.putText(annotated, f"collected: {brain.collected_count} | time: {mins}:{secs:02d}",
                (20, 175), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.putText(annotated, f"motors L:{left_speed} R:{right_speed}",
                (20, 215), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 180, 255), 2)

    telem = link.get_telemetry()
    if telem:
        cv2.putText(annotated,
                    f"STM: bat={telem.battery_v:.1f}V L={telem.speed_left} "
                    f"R={telem.speed_right} age={link.telemetry_age():.1f}s",
                    (20, 255), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 100), 2)
    else:
        cv2.putText(annotated, "STM: NO TELEMETRY",
                    (20, 255), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    return annotated, boundary_frame


def control_loop():
    """Крутит control_step() до остановки программы."""
    global _frame_seq
    while not _stop_event.is_set():
        try:
            main_frame, boundary_frame = control_step()
        except Exception:
            # Сбой одного кадра не должен убить луноход, но и ехать вслепую нельзя.
            link.emergency_stop()
            traceback.print_exc()
            time.sleep(0.5)
            continue
        with _frames_cond:
            _frames["video"] = main_frame
            if boundary_frame is not None:
                _frames["boundary"] = boundary_frame
            _frame_seq += 1
            _frames_cond.notify_all()


# ─────────────────────── Веб-интерфейс (только просмотр) ─────────────────────

def mjpeg_stream(name):
    """Отдаёт браузеру последние кадры потока управления как MJPEG."""
    last_seq = -1
    while True:
        with _frames_cond:
            _frames_cond.wait_for(lambda: _frame_seq != last_seq, timeout=1.0)
            frame = _frames[name]
            last_seq = _frame_seq
        if frame is None:
            continue
        ok, buffer = cv2.imencode('.jpg', frame)
        if not ok:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


@app.route('/video')
def video():
    return Response(mjpeg_stream("video"),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/boundary')
def boundary_video():
    return Response(mjpeg_stream("boundary"),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/calibrate')
def calibrate():
    # пере-калибровать камеру под текущий свет (наведи на белый/серый лист)
    locked = auto_calibrate(picam2)
    return f"Калибровка обновлена: {locked}"


@app.route('/restart')
def restart():
    # заново запустить раунд (сбросить таймер и состояние)
    with brain_lock:
        brain.start()
    return "Раунд перезапущен"


@app.route('/')
def index():
    return '''
    <html>
        <body>
            <h1>Луноход — двойное зрение</h1>
            <div style="display:flex; gap:10px;">
                <div>
                    <h3>Основная камера (ресурсы)</h3>
                    <img src="/video" width="640">
                </div>
                <div>
                    <h3>Нижняя камера (граница)</h3>
                    <img src="/boundary" width="640">
                </div>
            </div>
            <p><a href="/calibrate">Пере-калибровать камеру</a></p>
            <p><a href="/restart">Перезапустить раунд</a></p>
        </body>
    </html>
    '''


if __name__ == "__main__":
    brain.start()   # таймер раунда пошёл — луноход начинает работать сразу
    control_thread = threading.Thread(target=control_loop, daemon=True)
    control_thread.start()
    try:
        app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
    finally:
        _stop_event.set()
        control_thread.join(timeout=2.0)
        link.emergency_stop()
        link.stop()
        boundary_cam.release()
        picam2.stop()
        print("[shutdown] motors stopped, cameras released")
