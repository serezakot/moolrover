"""
Конечный автомат ("мозг") лунохода.
Хранит текущее состояние, таймер раунда, и принимает решения с памятью.
Импортирует сенсорные функции из navigation.py.
"""

import time
from navigation import choose_target, target_angle, target_distance, _coords


# === Пороги решений ===
DEADZONE = 0.15           # доля ширины кадра: цель "по центру"
COLLECT_AREA = 0.10       # доля площади кадра: цель "близко, пора собирать"
BOUNDARY_LEVEL = 0.15     # доля черноты: граница поля

# === Тайминги ===
ROUND_DURATION = 8 * 60   # длительность раунда (сек)
RETURN_TIME = 6.5 * 60    # когда начинать возврат (сек) — запас на дорогу
TARGET_LOST_TIMEOUT = 2.0 # сколько ждать пропавшую цель (сек)
COLLECT_TIMEOUT = 3.0     # сколько находиться в COLLECT (сек)


class RoverBrain:
    """
    Состояния:
        SEARCH   — кручусь/еду, ищу синее
        APPROACH — вижу цель, еду к ней
        COLLECT  — цель рядом, захватываю
        RETURN   — еду домой к жёлтой яме
        DEPOSIT  — у ямы, сбрасываю
        AVOID    — вижу границу, отъезжаю
        DONE     — раунд окончен
    """

    def __init__(self):
        self.state = "SEARCH"
        self.prev_state = "SEARCH"     # для возврата из AVOID
        self.start_time = None         # засекаем при вызове start()
        self.target_lost_since = None  # когда потеряли цель в APPROACH
        self.collect_start = None      # когда вошли в COLLECT
        self.collected_count = 0       # сколько ресурсов собрали
        self.start_heading = None      # heading при старте (от IMU, потом)

    def start(self):
        """Вызвать один раз при старте раунда (кнопка START)."""
        self.start_time = time.time()
        self.state = "SEARCH"

    def elapsed(self):
        """Сколько секунд прошло с начала раунда."""
        if self.start_time is None:
            return 0
        return time.time() - self.start_time

    def time_left(self):
        """Сколько секунд осталось до конца раунда."""
        return max(0, ROUND_DURATION - self.elapsed())

    def update(self, detections, frame_width, frame_height,
               boundary_side=None, heading=None):
        """
        boundary_side — 'LEFT'/'RIGHT'/'CENTER'/None: с какой стороны граница.
        """
        if self.start_time is None:
            return "STOP"

        if self.state == "DONE":
            return "STOP"

        # --- Таймер RETURN ---
        if self.elapsed() >= RETURN_TIME:
            if self.state not in ("RETURN", "DEPOSIT", "DONE", "AVOID"):
                self.state = "RETURN"

        # --- AVOID: граница впереди, высший приоритет ---
        if boundary_side is not None:
            if self.state not in ("AVOID", "DEPOSIT", "DONE"):
                self.prev_state = self.state
                self.state = "AVOID"

        target = choose_target(detections)

        if self.state == "SEARCH":
            return self._search(target, detections, frame_width, frame_height)
        elif self.state == "APPROACH":
            return self._approach(target, detections, frame_width, frame_height)
        elif self.state == "COLLECT":
            return self._collect(target)
        elif self.state == "RETURN":
            return self._return(heading)
        elif self.state == "DEPOSIT":
            return self._deposit()
        elif self.state == "AVOID":
            return self._avoid(boundary_side)

        return "STOP"

    # === Обработчики состояний ===

    def _search(self, target, detections, fw, fh):
        """SEARCH: кручусь, ищу синее. Нашёл → APPROACH."""
        if target is not None:
            self.state = "APPROACH"
            self.target_lost_since = None
            # Сразу обрабатываем APPROACH, не теряя кадр
            return self._approach(target, detections, fw, fh)
        return "SEARCH"

    def _approach(self, target, detections, fw, fh):
        """APPROACH: еду к цели. Близко → COLLECT. Потерял → жду, потом SEARCH."""

        # Цель потеряна — ждём TARGET_LOST_TIMEOUT, потом сдаёмся
        if target is None:
            if self.target_lost_since is None:
                self.target_lost_since = time.time()
            if time.time() - self.target_lost_since > TARGET_LOST_TIMEOUT:
                self.state = "SEARCH"
                self.target_lost_since = None
                return "SEARCH"
            # Инерция: продолжаем ехать вперёд, пока ждём
            return "FORWARD"

        # Цель видна — сбрасываем таймер потери
        self.target_lost_since = None

        x1, y1, x2, y2 = _coords(target)
        box_area = (x2 - x1) * (y2 - y1)
        frame_area = fw * fh
        closeness = box_area / frame_area

        # Цель достаточно близко → COLLECT
        if closeness >= COLLECT_AREA:
            self.state = "COLLECT"
            self.collect_start = time.time()
            return "COLLECT"

        # Центрирование: LEFT / RIGHT / FORWARD
        target_cx = (x1 + x2) / 2
        frame_cx = fw / 2
        offset = target_cx - frame_cx
        dead_px = DEADZONE * fw

        if offset < -dead_px:
            return "LEFT"
        elif offset > dead_px:
            return "RIGHT"
        else:
            return "FORWARD"

    def _collect(self, target):
        """COLLECT: захватываем. Цель пропала или таймаут → SEARCH."""
        elapsed = time.time() - self.collect_start

        # Цель исчезла (подобрали) или таймаут
        if target is None or elapsed > COLLECT_TIMEOUT:
            if target is None:
                # Скорее всего подобрали — считаем
                self.collected_count += 1
            self.state = "SEARCH"
            self.collect_start = None
            return "SEARCH"

        return "COLLECT"

    def _return(self, heading):
        """RETURN: едем домой. Пока без жёлтого детектора — разворот по IMU."""
        # TODO: когда добавим детекцию жёлтого:
        #   если вижу жёлтое → self.state = "DEPOSIT"; return "DEPOSIT"
        #
        # TODO: когда подключим IMU:
        #   развернуться на 180° от start_heading, ехать прямо
        #
        # Пока: просто возвращаем команду RETURN,
        # конкретное поведение (куда крутить) определят моторы + IMU
        return "RETURN"

    def _deposit(self):
        """DEPOSIT: сбрасываем ресурсы. → DONE (или SEARCH если время есть)."""
        # TODO: активировать серво MG995 для сброса
        self.collected_count = 0
        if self.elapsed() < RETURN_TIME:
            # Ещё есть время — едем за новыми
            self.state = "SEARCH"
            return "SEARCH"
        self.state = "DONE"
        return "STOP"

    def _avoid(self, boundary_side):
        """
        AVOID: граница впереди. Отворачиваем в сторону, где чёрного нет.
        Чёрное слева  → крутим вправо (TURN_RIGHT).
        Чёрное справа → крутим влево  (TURN_LEFT).
        Чёрное по центру → крутим в одну сторону (вправо) до выхода.
        Граница ушла  → возвращаемся в прежнее состояние.
        """
        if boundary_side is None:
            # чисто — граница ушла из кадра, продолжаем прежнее дело
            self.state = self.prev_state
            return "FORWARD"

        if boundary_side == "LEFT":
            return "TURN_RIGHT"
        elif boundary_side == "RIGHT":
            return "TURN_LEFT"
        else:  # CENTER
            return "TURN_RIGHT"