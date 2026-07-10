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
from stm32_link import STM32Link
from motor_controller import command_to_speed

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
# Вторая камера
BOUNDARY_CAM_INDEX = 0

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

# Мозг лунохода. start() — запускает таймер раунда.
brain = RoverBrain()
brain.start()

# --- Распознавание границы: яркое (ретрорефлектор) ИЛИ резкий контраст ---
BAND_HEIGHT = 0.30
BRIGHT_THRESHOLD = 5   # ярче медианы пола в N раз = ретрорефлектор
CONTRAST_KERNEL = 15     # размер окна для поиска резкого перепада яркости
CONTRAST_THRESHOLD = 250  # минимальный перепад яркости = "граница, а не тень"
ZONE_TRIGGER = 0.05
ABS_BRIGHT = 150


def boundary_zone(frame_bgr):
    """
    Ищет границу поля двумя способами:
    1. Яркие пиксели (ретрорефлектор) — основной сигнал.
    2. Резкий перепад яркости (контраст) — страховка вместо ABS_DARK.
    Тени дают плавное затемнение, низкий контраст → не тригерят.
    Граница поля = резкий скачок белое↔чёрное → высокий контраст → тригерит.
    """
    h, w = frame_bgr.shape[:2]
    band = frame_bgr[int(h * (1 - BAND_HEIGHT)):, :]
    gray = cv2.cvtColor(band, cv2.COLOR_BGR2GRAY).astype(float)

    floor = float(np.median(gray))

    # 1. Яркое — ретрорефлектор
    bright = (gray > (floor * BRIGHT_THRESHOLD)) & (gray > ABS_BRIGHT)

    # 2. Контраст — резкий перепад яркости (граница, не тень)
    gray_u8 = gray.astype(np.uint8)
    local_max = cv2.dilate(gray_u8, np.ones((CONTRAST_KERNEL, CONTRAST_KERNEL)))
    local_min = cv2.erode(gray_u8, np.ones((CONTRAST_KERNEL, CONTRAST_KERNEL)))
    contrast = (local_max.astype(float) - local_min.astype(float))
    high_contrast = contrast > CONTRAST_THRESHOLD

    # комбинация: нашли яркое ИЛИ резкий контраст
    boundary = bright | high_contrast

    third = w // 3
    left_frac   = float(boundary[:, :third].mean())
    center_frac = float(boundary[:, third:2 * third].mean())
    right_frac  = float(boundary[:, 2 * third:].mean())

    fracs = (left_frac, center_frac, right_frac)

    triggered = {
        "LEFT":   left_frac >= ZONE_TRIGGER,
        "CENTER": center_frac >= ZONE_TRIGGER,
        "RIGHT":  right_frac >= ZONE_TRIGGER,
    }

    if not any(triggered.values()):
        return None, fracs

    if triggered["CENTER"]:
        return "CENTER", fracs
    if left_frac > right_frac:
        return "LEFT", fracs
    return "RIGHT", fracs

# --- Детекция жёлтой ямы (deposit pit) по HSV ---
YELLOW_LOW = np.array([15, 80, 80])
YELLOW_HIGH = np.array([35, 255, 255])
YELLOW_TRIGGER = 0.05    # доля жёлтых пикселей в кадре, чтобы считать "я у ямы"


def detect_deposit(frame_bgr):
    """
    Ищет жёлтый цвет (яма) в кадре второй камеры.
    Возвращает (True, fraction) если жёлтого достаточно, иначе (False, fraction).
    """
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, YELLOW_LOW, YELLOW_HIGH)
    fraction = float(mask.mean()) / 255.0
    return fraction >= YELLOW_TRIGGER, fraction


def generate_boundary_frames():
    """Стрим второй камеры с визуальной обводкой зон границы."""
    while True:
        ok, frame2 = boundary_cam.read()
        if not ok:
            continue
        frame2 = cv2.flip(frame2, -1)

        h, w = frame2.shape[:2]
        b_side, b_fracs = boundary_zone(frame2)

        # рисуем три зоны в нижней полосе
        band_top = int(h * (1 - BAND_HEIGHT))
        third = w // 3

        # полупрозрачные прямоугольники по зонам: зелёный = чисто, красный = граница
        overlay = frame2.copy()
        colors = [
            (0, 0, 255) if b_fracs[0] >= ZONE_TRIGGER else (0, 255, 0),  # LEFT
            (0, 0, 255) if b_fracs[1] >= ZONE_TRIGGER else (0, 255, 0),  # CENTER
            (0, 0, 255) if b_fracs[2] >= ZONE_TRIGGER else (0, 255, 0),  # RIGHT
        ]
        cv2.rectangle(overlay, (0, band_top), (third, h), colors[0], -1)
        cv2.rectangle(overlay, (third, band_top), (2 * third, h), colors[1], -1)
        cv2.rectangle(overlay, (2 * third, band_top), (w, h), colors[2], -1)
        cv2.addWeighted(overlay, 0.3, frame2, 0.7, 0, frame2)

        # белые линии-разделители зон
        cv2.line(frame2, (third, band_top), (third, h), (255, 255, 255), 1)
        cv2.line(frame2, (2 * third, band_top), (2 * third, h), (255, 255, 255), 1)
        cv2.line(frame2, (0, band_top), (w, band_top), (255, 255, 255), 1)

        # подписи зон с процентом черноты
        labels = ["L", "C", "R"]
        for i, (lbl, frac) in enumerate(zip(labels, b_fracs)):
            cx = i * third + third // 2
            cv2.putText(frame2, f"{lbl}:{frac:.2f}",
                        (cx - 30, band_top + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # общий вердикт
        verdict = b_side if b_side else "CLEAR"
        color = (0, 0, 255) if b_side else (0, 255, 0)
        cv2.putText(frame2, f"BOUNDARY: {verdict}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        
        # показываем жёлтый детект
        _, yellow_frac = detect_deposit(frame2)
        yellow_color = (0, 255, 255) if yellow_frac >= YELLOW_TRIGGER else (200, 200, 200)
        cv2.putText(frame2, f"YELLOW: {yellow_frac:.2f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, yellow_color, 2)

        _, buffer = cv2.imencode('.jpg', frame2)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

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
        
        
        
        # кадр со второй камеры (граница + яма)
        ok, boundary_frame = boundary_cam.read()
        if ok:
            boundary_frame = cv2.flip(boundary_frame, -1)   # поворот на 180°
            b_side, b_fracs = boundary_zone(boundary_frame)
            at_deposit, yellow_frac = detect_deposit(boundary_frame)
        else:
            b_side, b_fracs = None, (0.0, 0.0, 0.0)
            at_deposit, yellow_frac = False, 0.0
            
        command = brain.update(detections, w, h, boundary_side=b_side, heading=None, at_deposit = at_deposit)

        # отправляем команду на моторы
        left_speed, right_speed = command_to_speed(command)
        link.set_speed(left_speed, right_speed)
        
        angle = target_angle(detections, w)
        dist = target_distance(detections, w)
        
        telem = link.get_telemetry()
        telem_age = link.telemetry_age()
        if telem:
            cv2.putText(
                annotated,
                f"STM: bat={telem.battery_v:.1f}V L={telem.speed_left} R={telem.speed_right} age={telem_age:.1f}s",
                (20, 255), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 100), 2
            )
        else:
            cv2.putText(
                annotated, "STM: NO TELEMETRY",
                (20, 255), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2
            )

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
        
        cv2.putText(
            annotated,
            f"motors L:{left_speed} R:{right_speed}",
            (20, 215), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 180, 255), 2
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

@app.route('/boundary')
def boundary_video():
    return Response(generate_boundary_frames(),
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
    try:
        app.run(host="0.0.0.0", port=5000, debug=False)
    finally:
        link.emergency_stop()
        link.stop()
        boundary_cam.release()
        print("[shutdown] motors stopped, cameras released")