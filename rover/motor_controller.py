"""
Переводчик команд автомата в скорости моторов.
Команда (строка) → (left_speed, right_speed) для STM32Link.

Скорости подобраны консервативно для первых тестов.
Калибруй на реальном поле: если луноход вялый — поднимай BASE,
если дёргается — опускай TURN.
"""

# --- Скорости (0..100), подбираются на тесте ---
BASE_SPEED = 35        # прямолинейная езда
STEER_FAST = 35        # внешнее колесо при довороте
STEER_SLOW = 15        # внутреннее колесо при довороте
TURN_SPEED = 30        # разворот на месте (AVOID, SEARCH)
COLLECT_SPEED = 20     # медленный подъезд к ресурсу
RETURN_SPEED = 30      # возврат к яме

# Маппинг команда → (left, right)
COMMAND_MAP = {
    "FORWARD":    ( BASE_SPEED,    BASE_SPEED),
    "LEFT":       ( STEER_SLOW,    STEER_FAST),    # доворот к цели влево
    "RIGHT":      ( STEER_FAST,    STEER_SLOW),    # доворот к цели вправо
    "SEARCH":     ( TURN_SPEED,   -TURN_SPEED),    # кручусь на месте по часовой
    "TURN_LEFT":  (-TURN_SPEED,    TURN_SPEED),    # разворот от границы влево
    "TURN_RIGHT": ( TURN_SPEED,   -TURN_SPEED),    # разворот от границы вправо
    "COLLECT":    ( COLLECT_SPEED,  COLLECT_SPEED), # медленно вперёд
    "RETURN":     ( RETURN_SPEED,   RETURN_SPEED),  # пока прямо (IMU потом)
    "DEPOSIT":    ( 0,              0),             # стоим, серво работает
    "STOP":       ( 0,              0),
    "AVOID":      ( TURN_SPEED,   -TURN_SPEED),    # на всякий случай
}


def command_to_speed(command: str) -> tuple:
    """Команда автомата → (left_speed, right_speed)."""
    return COMMAND_MAP.get(command, (0, 0))