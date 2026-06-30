# Moonrover 🌙

An autonomous **lunar rover** built around a **Raspberry Pi 5**. The rover uses
computer vision to find resource samples on the ground, drives up to them,
collects them with a manipulator, and keeps searching — while staying inside a
marked boundary and avoiding obstacles.

> Educational / student project. Work in progress.

---

## 1. What it does (autonomy pipeline)

```
        SEE                 DRIVE                  COLLECT              LEAVE
   ┌───────────┐      ┌────────────────┐      ┌──────────────┐      ┌──────────┐
   │ camera +  │ ───► │ centre target, │ ───► │ stop, lower, │ ───► │ back off │
   │  YOLO      │      │ approach it    │      │ grip, lift   │      │ & search │
   └───────────┘      └────────────────┘      └──────────────┘      └──────────┘
        │                     ▲                                           │
        │  no target          │  obstacle / boundary in front             │
        ▼                     └───────────────── AVOID ◄──────────────────┘
   systematic search
   (scan 360° → relocate)
```

The "brain" is a finite-state machine: **SEARCH → APPROACH → COLLECT → DEPART**,
with **AVOID** overriding everything for safety.

---

## 2. Hardware / components

| Component | Role | Status |
|-----------|------|--------|
| **Raspberry Pi 5** | Main computer: vision, decision-making | In use |
| **Pi Camera (Camera Module 3 Wide, 102° HFOV)** | Primary sensor — finds samples | In use |
| **STM32F103 "Blue Pill" (Cortex-M3)** | Low-level motor & manipulator controller, talks to Pi over UART | Firmware = UART loopback so far |
| **Differential drive (2 motors / tracks)** | Movement | Planned (protocol ready) |
| **Manipulator / gripper** | Picks up samples | Planned (interface + protocol ready) |
| **Range sensors (ultrasonic / ToF)** | Obstacle detection | Planned (interface ready) |
| Boundary line on the ground | Keeps rover inside the arena | Detected from the camera frame |

Pi ↔ STM32 link: **UART, 9600 baud, 8N1**, `/dev/ttyAMA0` on the Pi side.

---

## 3. Technology stack

- **Python 3** — all high-level logic on the Pi
- **Ultralytics YOLO** (`yolo11n.pt` for tests, custom `best.pt` for the real model) — object detection & tracking (ByteTrack)
- **OpenCV** + **Picamera2** + **libcamera** — capture and image processing
- **Flask** — live MJPEG video/debug stream
- **C (bare-metal, `arm-none-eabi-gcc`)** — STM32 firmware (direct register access)
- **pyserial** — Pi ↔ STM32 UART

---

## 4. Repository structure

```
moolrover/
├── navigation.py          # "brain": decide() + target geometry + confidence threshold
├── track.py               # live YOLO tracking + boundary detection + Flask stream
├── live_detect.py         # camera + YOLO, prints FPS and detected objects
├── detect_test.py         # YOLO on a single image (sanity check)
├── test_yolo.py           # loads the YOLO model (smoke test)
│
├── rover/                 # autonomous behaviour layer (see → drive → collect)
│   ├── mission.py         #   finite-state machine (SEARCH/APPROACH/COLLECT/DEPART/AVOID)
│   ├── search.py          #   systematic target search when nothing is visible
│   ├── perception.py      #   YOLO detections (navigation.py) -> TargetView
│   ├── hardware.py        #   hardware interfaces (Drive, Manipulator, sensors) + types
│   ├── uart_link.py       #   UART protocol to STM32 (motors + manipulator)
│   ├── simulation.py      #   2D physics + stub sensors to run without hardware
│   ├── app.py             #   entry point (sim demo / real-robot skeleton)
│   └── README.md          #   detailed architecture of this package
│
├── uart_version_1/        # STM32 firmware (currently a UART loopback)
│   ├── main.c             #   USART1 echo, direct register access
│   ├── startup.c          #   reset handler / vector table
│   ├── linker_script.ld   #   memory layout
│   ├── Makefile           #   arm-none-eabi-gcc build
│   └── uart_lootback.py   #   Pi-side loopback test (pyserial)
│
└── tests/                 # pure-Python tests (no camera / YOLO / hardware needed)
    ├── test_navigation.py #   confidence threshold + decisions
    ├── test_mission.py    #   full mission on the simulator
    ├── test_search.py     #   search planner
    └── test_uart.py       #   UART frame encoding
```

---

## 5. How the brain works

### Detection & confidence threshold (`navigation.py`)
YOLO returns boxes with a confidence score. Detections are passed as
`(x1, y1, x2, y2, conf)`; anything **below `CONFIDENCE_THRESHOLD` (0.5)** is
dropped *before* a target is chosen, so the rover ignores noise, glare and weak
guesses. The threshold is tunable, and the old `(x1, y1, x2, y2)` format still
works (treated as confidence = 1.0).

Key parameters:

| Constant | Meaning | Default |
|----------|---------|---------|
| `CONFIDENCE_THRESHOLD` | minimum YOLO confidence to trust a detection | `0.5` |
| `DEADZONE` | central band where the target counts as "centred" | `0.15` |
| `COLLECT_AREA` | box area fraction at which the target is "close enough" | `0.10` |
| `BOUNDARY_LEVEL` | dark fraction in front that triggers AVOID | `0.15` |
| `CAMERA_HFOV` | camera horizontal field of view (deg) | `102` |

### Boundary detection (`track.py`)
The bottom strip of the frame is checked for pixels much darker than the floor
(or below an absolute threshold). A high "dark" fraction means a boundary edge
is right in front → the brain returns **AVOID**.

### State machine (`rover/mission.py`)
| State | Behaviour |
|-------|-----------|
| `SEARCH` | No target → systematic search (`search.py`). |
| `APPROACH` | Target visible → proportional control keeps it centred while driving in; steers around obstacles. |
| `COLLECT` | Target close & centred → stop, lower, grip, lift; only counts a **confirmed** grip. |
| `DEPART` | Sample taken → back up and turn away, then search again. |
| `AVOID` | Boundary/obstacle in front → highest priority; back off and turn to the clear side. |

### Systematic search (`rover/search.py`)
When no target is visible the rover does **not** spin randomly:
1. **Re-acquire** — turn back toward where the target was last seen.
2. **Scan** — full 360° look-around on the spot.
3. **Relocate** — move to a new vantage point along an **expanding spiral**,
   steering around obstacles, then scan again.

Rotation is measured from real odometry, and visited cells are remembered, so the
search expands outward instead of looping in place.

---

## 6. UART protocol (Pi → STM32) — `rover/uart_link.py`

Frame: `0xAA  CMD  LEN  payload[LEN]  CRC8`
(CRC-8, polynomial `0x07`, computed over `CMD + LEN + payload`).

| CMD | Name | Payload |
|-----|------|---------|
| `0x01` | DRIVE | `int8 left, int8 right` (−100…100 % motor power) |
| `0x02` | MANIP | `uint8 action` (0 = release, 1 = grip, 2 = lower, 3 = lift) |
| `0x03` | STOP | — |
| `0x10` | PING | — |

`DriveCommand(linear, angular)` is normalised to `[-1, 1]`; `to_differential()`
converts it into left/right wheel speeds. The Python side is ready; the STM32
firmware still needs to parse these frames and drive the motors (today it just
echoes bytes back — the loopback in `uart_version_1/`).

---

## 7. Running it

### Simulation — logic only, no hardware needed
```bash
python3 -m rover.app          # prints state transitions; "собрано 2 из 2" = success
```

### Tests — no camera / YOLO / hardware
```bash
python3 tests/test_navigation.py
python3 tests/test_mission.py
python3 tests/test_search.py
python3 tests/test_uart.py
```

### On the Raspberry Pi (with camera + YOLO)
```bash
python3 live_detect.py        # camera + YOLO, prints FPS and objects
python3 track.py              # tracking + boundary + brain, web stream on :5000
```
Open `http://<pi-ip>:5000/` to see the annotated stream (command, target angle,
boundary level, detection confidence vs threshold).

### STM32 firmware
```bash
cd uart_version_1
make                          # builds firmware.bin (arm-none-eabi-gcc)
# flash firmware.bin to the board; test the link with:
python3 uart_lootback.py
```

---

## 8. Wiring up the real rover

The behaviour layer talks only to the interfaces in `rover/hardware.py`, so the
same state machine runs in simulation and on the real robot — you just swap the
implementations:

```python
from rover.mission import RoverMission
from rover.uart_link import Stm32Link

link = Stm32Link("/dev/ttyAMA0", 9600)     # drive + manipulator over UART
mission = RoverMission(
    drive=link, manipulator=link,
    target_sensor=...,    # camera + YOLO -> perception.build_target_view
    range_sensor=...,     # ultrasonic / ToF
    boundary_sensor=...,  # track.boundary_level on the frame
)
mission.run()
```

---

## 9. Status & roadmap

**Done**
- YOLO detection + tracking, live debug stream
- Confidence threshold filtering
- Boundary detection from the camera
- Full autonomy logic (search → approach → collect → depart → avoid) running in simulation
- UART protocol definition + Python side
- Test suite (pure Python)

**Next**
- STM32 firmware: parse the UART frames, drive motors (PWM) and the manipulator
- `YoloTargetSensor` adapter: feed real camera detections into the mission
- Obstacle range sensors (ultrasonic / ToF) wired in
- Tune thresholds and control gains on the real rover
- Optional: deliver collected samples to a drop-off / base

---

## 10. Team

Student project. Built around Raspberry Pi 5, Python, OpenCV and YOLO, with an
STM32 motor controller over UART.