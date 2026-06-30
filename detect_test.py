from ultralytics import YOLO

model = YOLO("yolo11n.pt")

results = model("test.jpg")

for box in results[0].boxes:
    cls = int(box.cls[0])
    conf = float(box.conf[0])

    print(
        f"{results[0].names[cls]} "
        f"{conf:.2f}"
    )