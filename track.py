from pathlib import Path

from flask import Flask, Response
from picamera2 import Picamera2
from ultralytics import YOLO
import cv2
from libcamera import controls
import numpy as np

# сенсорные функции ("глаза")
from navigation import target_angle, target_distance
# автонастройка камеры под освещение + проверка синего по HSV
from camera_setup import auto_calibrate, recalibrate_on_demand, blue_fraction
# автомат ("мозг" с памятью)
from state_machine import RoverBrain


# --- Фильтр "реально ли синий" -------------------------------------------
USE_BLUE_FILTER = True     # отсекать боксы YOLO, в которых мало синего
BLUE_MIN_FRACTION = 0.20   # минимальная доля синих пикселей в боксе (0..1)


app = Flask(__name__)

# Камера
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (1280, 720)})
picam2.configure(config)
picam2.start()

# Автокалибровка баланса белого / экспозиции под ТЕКУЩИЙ свет.
# Наведи камеру на нейтральный фон — белый/серый лист — на момент старта.
auto_calibrate(picam2, settle_seconds=2.0)

# YOLO модель
PROJECT_ROOT = Path(__file__).resolve().parent
MODEL_CANDIDATES = [PROJECT_ROOT / "best.pt", PROJECT_ROOT / "yolo11n.pt"]
MODEL_PATH = next((p for p in MODEL_CANDIDATES if p.exists()), None)
if MODEL_PATH is None:
    raise FileNotFoundError(
        f"YOLO weights not found. Looked for: {[str(p) for p in MODEL_CANDIDATES]}"
    )
model = YOLO(str(MODEL_PATH))

# Мозг лунохода. start() — запускает таймер раунда.
brain = RoverBrain()
brain.start()

# --- Распознавание границы по зонам: слева / по центру / справа ---
DARK_MARGIN = 35       # насколько темнее медианы пола = "граница"
ABS_DARK = 45          # абсолютный потолок: темнее этого = граница в любом случае
BAND_HEIGHT = 0.30     # какую часть кадра снизу считаем "землёй перед луноходом"
ZONE_TRIGGER = 0.15    # доля черноты в зоне, чтобы считать её "границей"


def boundary_zone(frame_bgr):
    """
    Смотрит на нижнюю полосу кадра и определяет, С КАКОЙ СТОРОНЫ граница.
    Возвращает:
        'LEFT'   — чёрное слева по курсу
        'RIGHT'  — чёрное справа по курсу
        'CENTER' — чёрное прямо перед носом
        None     — чисто, границы нет
    Плюс возвращает доли черноты по зонам (для отладки на стриме).
    """
    h, w = frame_bgr.shape[:2]
    band = frame_bgr[int(h * (1 - BAND_HEIGHT)):, :]
    gray = cv2.cvtColor(band, cv2.COLOR_BGR2GRAY)

    floor = float(np.median(gray))          # типичная яркость пола
    threshold = max(floor - DARK_MARGIN, ABS_DARK)
    dark = (gray < threshold)               # маска "тёмных" пикселей

    # делим полосу на три вертикальные зоны
    third = w // 3
    left_frac   = float(dark[:, :third].mean())
    center_frac = float(dark[:, third:2 * third].mean())
    right_frac  = float(dark[:, 2 * third:].mean())

    fracs = (left_frac, center_frac, right_frac)

    # какая зона "сработала" (черноты больше порога)
    triggered = {
        "LEFT": left_frac >= ZONE_TRIGGER,
        "CENTER": center_frac >= ZONE_TRIGGER,
        "RIGHT": right_frac >= ZONE_TRIGGER,
    }

    if not any(triggered.values()):
        return None, fracs

    # приоритет: центр важнее (прямо перед носом), потом — где черноты больше
    if triggered["CENTER"]:
        return "CENTER", fracs
    if left_frac > right_frac:
        return "LEFT", fracs
    return "RIGHT", fracs


def generate_frames():
    while True:
        frame = picam2.capture_array()

        # RGBA -> BGR
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

        # YOLO tracking
        results = model.track(
            frame,
            persist=True,
            tracker="bytetrack.yaml",
            verbose=False
        )

        annotated = results[0].plot()

        # достаём координаты боксов; каждый проверяем по доле синего
        detections = []
        best_blue = 0.0
        boxes = results[0].boxes
        if boxes is not None and boxes.xyxy is not None:
            for x1, y1, x2, y2 in boxes.xyxy.tolist():
                bf = blue_fraction(frame, (x1, y1, x2, y2))
                best_blue = max(best_blue, bf)
                if USE_BLUE_FILTER and bf < BLUE_MIN_FRACTION:
                    continue                 # мало синего -> не наш образец
                detections.append((x1, y1, x2, y2))

        h, w = annotated.shape[:2]
        
        
        """
        frame помкнять надо будет когда купим вторую камеру, бо сейчас границы на основной 
        """
        b_side, b_fracs = boundary_zone(frame) 
        command = brain.update(detections, w, h, boundary_side=b_side, heading=None)

        angle = target_angle(detections, w)
        dist = target_distance(detections, w)

        # Строка состояния автомата + команда
        label = f"{brain.state} | {command}"
        if angle is not None:
            label += f"  ({angle:+.0f} deg)"
        if dist is not None:
            label += f"  {dist:.2f}m"

        cv2.putText(
            annotated, label, (20, 50),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3
        )
        cv2.putText(
            annotated,
            f"border: {b_side} L{b_fracs[0]:.2f} C{b_fracs[1]:.2f} R{b_fracs[2]:.2f}",
            (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
        )
        cv2.putText(
            annotated, f"blue: {best_blue:.2f} / min {BLUE_MIN_FRACTION:.2f}", (20, 135),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2
        )
        # собрано + оставшееся время раунда
        mins, secs = divmod(int(brain.time_left()), 60)
        cv2.putText(
            annotated, f"collected: {brain.collected_count} | time: {mins}:{secs:02d}",
            (20, 175), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2
        )

        # encode JPEG
        _, buffer = cv2.imencode('.jpg', annotated)
        frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video')
def video():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/calibrate')
def calibrate():
    # пере-калибровать камеру под текущий свет (наведи на белый/серый лист)
    locked = recalibrate_on_demand(picam2)
    return f"Калибровка обновлена: {locked}"


@app.route('/restart')
def restart():
    # заново запустить раунд (сбросить таймер и состояние)
    brain.start()
    return "Раунд перезапущен"


@app.route('/')
def index():
    return '''
    <html>
        <body>
            <h1>YOLO Camera Stream</h1>
            <img src="/video" width="640">
            <p><a href="/calibrate">Пере-калибровать камеру</a></p>
            <p><a href="/restart">Перезапустить раунд</a></p>
        </body>
    </html>
    '''


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)