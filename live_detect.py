from pathlib import Path

from picamera2 import Picamera2
from ultralytics import YOLO
import cv2
import time

picam2 = Picamera2()
picam2.start()

PROJECT_ROOT = Path(__file__).resolve().parent
MODEL_CANDIDATES = [PROJECT_ROOT / "best.pt", PROJECT_ROOT / "yolo11n.pt"]
MODEL_PATH = next((p for p in MODEL_CANDIDATES if p.exists()), None)
if MODEL_PATH is None:
    raise FileNotFoundError(
        f"YOLO weights not found. Looked for: {[str(p) for p in MODEL_CANDIDATES]}"
    )
model = YOLO(str(MODEL_PATH))

while True:
    frame = picam2.capture_array()

    # RGBA -> RGB
    frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)

    start = time.time()

    results = model(frame, verbose=False)

    elapsed = time.time() - start

    objects = []

    for box in results[0].boxes:
        cls = int(box.cls[0])
        conf = float(box.conf[0])

        objects.append(
            f"{results[0].names[cls]} ({conf:.2f})"
        )

    fps = 1 / elapsed

    print(f"FPS: {fps:.2f} | {objects}")