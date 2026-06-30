from flask import Flask, Response
from picamera2 import Picamera2
from ultralytics import YOLO
import cv2
from navigation import decide, target_angle
from libcamera import controls
import numpy as np

from navigation import decide   # наш "мозг"





app = Flask(__name__)

# Камера
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (1280, 720)})
picam2.configure(config)
picam2.start()



# Баланс белого под студийный белый свет
# Ручной баланс белого — убираем синеву напрямую
picam2.set_controls({
    "AwbEnable": False,
    "ColourGains": (2.0, 1.2)
})

# YOLO модель
model = YOLO("best.pt")

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

        #  достаём координаты боксов и спрашиваем "мозг"
        detections = []
        boxes = results[0].boxes
        if boxes is not None and boxes.xyxy is not None:
            for x1, y1, x2, y2 in boxes.xyxy.tolist():
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

@app.route('/')
def index():
    return '''
    <html>
        <body>
            <h1>YOLO Camera Stream</h1>
            <img src="/video" width="640">
        </body>
    </html>
    '''

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)