from pathlib import Path

from flask import Flask, Response
from picamera2 import Picamera2
from ultralytics import YOLO
import cv2
from navigation import decide, target_angle
from libcamera import controls
import numpy as np

# автонастройка камеры под освещение + проверка синего по HSV
from camera_setup import auto_calibrate, recalibrate_on_demand, blue_fraction

from navigation import decide   # наш "мозг"


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
# (вместо ручного ColourGains, который душил синий и плыл при смене света).
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

# --- Распознавание границы: "намного темнее, чем пол вокруг" ---
DARK_MARGIN = 35       # насколько темнее медианы пола, чтобы считать "границей"
BAND_HEIGHT = 0.20     # нижние 25% кадра — "опасная полоса" перед луноходом

# --- Распознавание границы: темнее пола ИЛИ просто очень тёмное ---
DARK_MARGIN = 35       # насколько темнее медианы пола = "граница"
ABS_DARK = 45          # абсолютный потолок: темнее этого = граница в любом случае
BAND_HEIGHT = 0.10     # нижние 10% кадра — ближний край перед луноходом

def boundary_level(frame_bgr):
    """
    Доля "тёмных" пикселей в нижней полосе.
    Тёмный = заметно темнее пола ИЛИ темнее абсолютного порога.
    Второе условие спасает, когда чёрное залило всю полосу и медиана уехала.
    """
    h, w = frame_bgr.shape[:2]
    band = frame_bgr[int(h * (1 - BAND_HEIGHT)):, :]
    gray = cv2.cvtColor(band, cv2.COLOR_BGR2GRAY)

    floor = float(np.median(gray))          # типичная яркость пола
    relative = floor - DARK_MARGIN          # порог "темнее пола"
    threshold = max(relative, ABS_DARK)     # берём более надёжный из двух

    dark = (gray < threshold)
    return float(dark.mean())


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

        #  достаём координаты боксов и спрашиваем "мозг".
        #  каждый бокс проверяем по доле синего — отсекаем не-синие ложные.
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
        b_level = boundary_level(frame)              # доля черноты перед луноходом
        command = decide(detections, w, h, b_level)
        angle = target_angle(detections, w)

        # текст команды + угол
        label = f"CMD: {command}"
        if angle is not None:
            label += f"  ({angle:+.0f} deg)"

        cv2.putText(
            annotated, label, (20, 50),
            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3
        )
        # уровень черноты — чтобы подбирать порог
        cv2.putText(
            annotated, f"dark: {b_level:.2f}", (20, 95),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2
        )
        # доля синего у самого "синего" бокса — чтобы подбирать BLUE_MIN_FRACTION
        cv2.putText(
            annotated, f"blue: {best_blue:.2f} / min {BLUE_MIN_FRACTION:.2f}", (20, 135),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2
        )
        # ------------------------------------------------------------

        # encode JPEG
        _, buffer = cv2.imencode('.jpg', annotated)
        frame = buffer.tobytes()

        # MJPEG stream format
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

@app.route('/')
def index():
    return '''
    <html>
        <body>
            <h1>YOLO Camera Stream</h1>
            <img src="/video" width="640">
            <p><a href="/calibrate">Пере-калибровать камеру</a></p>
        </body>
    </html>
    '''

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)
