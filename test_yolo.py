from pathlib import Path

from ultralytics import YOLO

print("Loading model...")

PROJECT_ROOT = Path(__file__).resolve().parent
MODEL_CANDIDATES = [PROJECT_ROOT / "best.pt", PROJECT_ROOT / "yolo11n.pt"]
MODEL_PATH = next((p for p in MODEL_CANDIDATES if p.exists()), None)
if MODEL_PATH is None:
    raise FileNotFoundError(
        f"YOLO weights not found. Looked for: {[str(p) for p in MODEL_CANDIDATES]}"
    )
model = YOLO(str(MODEL_PATH))

print("Model loaded!")


