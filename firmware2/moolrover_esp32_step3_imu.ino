/*
  MOOLROVER — ESP32-WROOM-32D / Arduino core 3.x
  ШАГ 3: всё из шага 2 + IMU (гироскоп MPU9250 из GY-91).

  IMU даёт:
    - heading (курс) в градусах от стартовой позиции (гироскоп Z, интеграция)
    - pos_x, pos_y — грубые координаты (dead reckoning: одометрия + heading)

  ВАЖНО: rover_types.h должен лежать в ТОЙ ЖЕ ПАПКЕ (обновлённый, 16-байтная телеметрия).
*/

#include <Arduino.h>
#include <Wire.h>
#include "rover_types.h"

// ================= ПИНЫ =================
#define PIN_PWM_L    25
#define PIN_PWM_R    26
#define PIN_DIR_L    27
#define PIN_DIR_R    14

#define PIN_SERVO_0  32   // ковш
#define PIN_SERVO_1  33   // кузов

#define PIN_ENC_L    18
#define PIN_ENC_R    19

#define PIN_UART_RX  16   // <- TX Малинки
#define PIN_UART_TX  17   // -> RX Малинки

#define PIN_SDA      21   // I2C для GY-91
#define PIN_SCL      22

HardwareSerial PiSerial(2);

// ================= ШИМ =================
#define MOTOR_FREQ   24000
#define MOTOR_RES    10
#define MOTOR_MAX    1023

#define SERVO_FREQ   50
#define SERVO_RES    16

// ================= ФИЗИКА =================
#define TICKS_PER_REV        120.0f
#define WHEEL_CIRCUMFERENCE  0.38683f   // метров

// !!! ПОДСТАВЬ ИЗМЕРЕННОЕ ЗНАЧЕНИЕ (полный ШИМ, установившаяся скорость).
#define MAX_SPEED_MPS        1.025f

// ================= IMU (MPU9250) =================
#define MPU9250_ADDR         0x68

// регистры
#define REG_WHO_AM_I         0x75
#define REG_PWR_MGMT_1       0x6B
#define REG_PWR_MGMT_2       0x6C
#define REG_GYRO_CONFIG      0x1B
#define REG_ACCEL_CONFIG     0x1C
#define REG_CONFIG           0x1A
#define REG_SMPLRT_DIV       0x19
#define REG_GYRO_ZOUT_H      0x47   // Z-ось гироскопа (старший байт)

// ±250 °/с  →  131 LSB/(°/с).  Хватит для ровера с запасом.
#define GYRO_RANGE_250       0x00
#define GYRO_SENSITIVITY     131.0f

// калибровка: сколько сэмплов усреднить при старте (ровер ДОЛЖЕН стоять неподвижно!)
#define GYRO_CAL_SAMPLES     500
#define GYRO_CAL_DELAY_US    2000   // пауза между сэмплами ≈ 2 мс → ~1 сек калибровки

// ================= КОМАНДЫ =================
#define CMD_EMERGENCY_STOP   0x00
#define CMD_SET_SPEED        0x01
#define CMD_RESET_ODOMETER   0x02
#define CMD_TUNING_PID       0x03
#define CMD_SET_PERIPHERY    0x04
#define CMD_CONTROL_SERVOS   0x05
#define CMD_RESET_IMU        0x06   // НОВАЯ: сбросить heading и координаты в 0

#define COMMAND_TIMEOUT_MS   300

// ================= ЭНКОДЕРЫ =================
volatile int32_t encoder_tick_left  = 0;
volatile int32_t encoder_tick_right = 0;
volatile bool dir_left_forward  = true;
volatile bool dir_right_forward = true;

void IRAM_ATTR isr_enc_left(void)  { dir_left_forward  ? encoder_tick_left++  : encoder_tick_left--; }
void IRAM_ATTR isr_enc_right(void) { dir_right_forward ? encoder_tick_right++ : encoder_tick_right--; }

float speed_left_mps  = 0.0f;
float speed_right_mps = 0.0f;
float odometer_left   = 0.0f;
float odometer_right  = 0.0f;

float speed_left_pct  = 0.0f;
float speed_right_pct = 0.0f;

// ================= ЦЕЛИ =================
float target_speed_left  = 0.0f;
float target_speed_right = 0.0f;
float target_servo_angles[2] = {90.0f, 90.0f};

uint32_t last_packet_received_time = 0;

// ================= PID =================
pid_controller pid_left, pid_right;

void pid_init(pid_controller* pid, float Kp, float Ki, float Kd, float max_out, float min_out) {
  pid->Kp = Kp; pid->Ki = Ki; pid->Kd = Kd;
  pid->max_output = max_out;
  pid->min_output = min_out;
  pid->Integral = 0.0f;
  pid->previous_error = 0.0f;
}

float pid_compute(pid_controller* pid, float set_point, float feedback, float dt) {
  if (dt <= 0.0f) return 0.0f;
  float error = set_point - feedback;
  float derivative = (error - pid->previous_error) / dt;
  pid->Integral += error * dt;
  if (pid->Integral > pid->max_output)      pid->Integral = pid->max_output;
  else if (pid->Integral < pid->min_output) pid->Integral = pid->min_output;
  float output = error * pid->Kp + pid->Integral * pid->Ki + derivative * pid->Kd;
  if (output > pid->max_output)      output = pid->max_output;
  else if (output < pid->min_output) output = pid->min_output;
  pid->previous_error = error;
  return output;
}

// ================= МОТОРЫ =================
void init_motors(void) {
  pinMode(PIN_DIR_L, OUTPUT);
  pinMode(PIN_DIR_R, OUTPUT);
  ledcAttach(PIN_PWM_L, MOTOR_FREQ, MOTOR_RES);
  ledcAttach(PIN_PWM_R, MOTOR_FREQ, MOTOR_RES);
  ledcWrite(PIN_PWM_L, MOTOR_MAX);
  ledcWrite(PIN_PWM_R, MOTOR_MAX);
}

void drive_motors(float left_pwm, float right_pwm) {
  if (left_pwm >= 0.0f) { digitalWrite(PIN_DIR_L, HIGH); dir_left_forward = true; }
  else { digitalWrite(PIN_DIR_L, LOW); dir_left_forward = false; left_pwm = -left_pwm; }
  if (left_pwm > 100.0f) left_pwm = 100.0f;
  ledcWrite(PIN_PWM_L, MOTOR_MAX - (uint32_t)(left_pwm * (MOTOR_MAX / 100.0f)));

  if (right_pwm >= 0.0f) { digitalWrite(PIN_DIR_R, HIGH); dir_right_forward = true; }
  else { digitalWrite(PIN_DIR_R, LOW); dir_right_forward = false; right_pwm = -right_pwm; }
  if (right_pwm > 100.0f) right_pwm = 100.0f;
  ledcWrite(PIN_PWM_R, MOTOR_MAX - (uint32_t)(right_pwm * (MOTOR_MAX / 100.0f)));
}

// ================= СЕРВО =================
void init_servos(void) {
  ledcAttach(PIN_SERVO_0, SERVO_FREQ, SERVO_RES);
  ledcAttach(PIN_SERVO_1, SERVO_FREQ, SERVO_RES);
}

void set_servo_angle(uint8_t servo_id, float angle) {
  if (angle < 0.0f) angle = 0.0f;
  if (angle > 180.0f) angle = 180.0f;
  uint32_t width_us = 1000 + (uint32_t)(angle * (1000.0f / 180.0f));
  uint32_t duty = (uint32_t)((uint64_t)width_us * 65536ULL / 20000ULL);
  if (servo_id == 0)      ledcWrite(PIN_SERVO_0, duty);
  else if (servo_id == 1) ledcWrite(PIN_SERVO_1, duty);
}

// ================= ЭНКОДЕРЫ: init + расчёт =================
void init_encoders(void) {
  pinMode(PIN_ENC_L, INPUT_PULLUP);
  pinMode(PIN_ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L), isr_enc_left,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R), isr_enc_right, CHANGE);
}

void update_motors(float dt) {
  noInterrupts();
  int32_t ticks_l = encoder_tick_left;
  int32_t ticks_r = encoder_tick_right;
  encoder_tick_left = 0;
  encoder_tick_right = 0;
  interrupts();

  speed_left_mps  = (((float)ticks_l) / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE / dt;
  speed_right_mps = (((float)ticks_r) / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE / dt;

  odometer_left  += speed_left_mps  * dt;
  odometer_right += speed_right_mps * dt;

  speed_left_pct  = (speed_left_mps  / MAX_SPEED_MPS) * 100.0f;
  speed_right_pct = (speed_right_mps / MAX_SPEED_MPS) * 100.0f;

  float out_l = pid_compute(&pid_left,  target_speed_left,  speed_left_pct,  dt);
  float out_r = pid_compute(&pid_right, target_speed_right, speed_right_pct, dt);

  drive_motors(out_l, out_r);
}

// ================= IMU =================
float gyro_bias_z   = 0.0f;    // смещение нуля, °/с (определяется при калибровке)
float heading_deg   = 0.0f;    // курс от старта, градусы (+ по часовой если смотреть сверху)
float pos_x_m       = 0.0f;    // координаты от старта, метры
float pos_y_m       = 0.0f;
uint8_t imu_ok      = 0;       // 0=не инициализирован, 1=ок, 2=ошибка

// ---------- низкоуровневые I2C хелперы ----------

void imu_write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t imu_read_reg(uint8_t reg) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

int16_t imu_read_gyro_z(void) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(REG_GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0;
  int16_t val = (int16_t)(Wire.read() << 8);
  val |= Wire.read();
  return val;
}

// ---------- init + калибровка ----------

void init_imu(void) {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);          // 400 кГц fast mode
  delay(10);

  // проверяем WHO_AM_I
  uint8_t who = imu_read_reg(REG_WHO_AM_I);
  // MPU9250 = 0x71, MPU6500 = 0x70, MPU9255 = 0x73
  if (who != 0x71 && who != 0x70 && who != 0x73) {
    Serial.print("IMU WHO_AM_I=0x"); Serial.print(who, HEX);
    Serial.println(" — неизвестный чип, продолжаю без IMU");
    imu_ok = 2;
    return;
  }

  // сброс
  imu_write_reg(REG_PWR_MGMT_1, 0x80);   // device reset
  delay(100);

  // wake up, автовыбор лучших часов
  imu_write_reg(REG_PWR_MGMT_1, 0x01);
  delay(10);

  // включить все оси акселя и гироскопа
  imu_write_reg(REG_PWR_MGMT_2, 0x00);

  // DLPF = 3 (41 Гц bandwidth, 5.9 мс delay) — убирает ВЧ шум гироскопа
  imu_write_reg(REG_CONFIG, 0x03);

  // sample rate divider: 1 кГц / (1+9) = 100 Гц внутренний rate
  imu_write_reg(REG_SMPLRT_DIV, 9);

  // гироскоп ±250 °/с
  imu_write_reg(REG_GYRO_CONFIG, GYRO_RANGE_250);

  // акселерометр ±2g (не используем пока, но пусть будет настроен)
  imu_write_reg(REG_ACCEL_CONFIG, 0x00);

  delay(50);   // дать DLPF устаканиться

  // ---------- калибровка смещения гироскопа ----------
  // РОВЕР ДОЛЖЕН СТОЯТЬ НЕПОДВИЖНО ВО ВРЕМЯ КАЛИБРОВКИ!
  Serial.print("IMU: калибровка гироскопа (");
  Serial.print(GYRO_CAL_SAMPLES);
  Serial.print(" сэмплов)... ");

  float sum_z = 0.0f;
  int good = 0;
  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    int16_t raw = imu_read_gyro_z();
    sum_z += (float)raw;
    good++;
    delayMicroseconds(GYRO_CAL_DELAY_US);
  }

  if (good > 0) {
    gyro_bias_z = (sum_z / (float)good) / GYRO_SENSITIVITY;   // °/с
  }

  heading_deg = 0.0f;
  pos_x_m = 0.0f;
  pos_y_m = 0.0f;
  imu_ok = 1;

  Serial.print("готово. bias_z=");
  Serial.print(gyro_bias_z, 4);
  Serial.println(" °/с");
}

// ---------- обновление (вызывается каждые 20 мс, вместе с моторами) ----------

void update_imu(float dt) {
  if (imu_ok != 1) return;

  // 1. Читаем угловую скорость Z
  int16_t raw_z = imu_read_gyro_z();
  float rate_dps = ((float)raw_z / GYRO_SENSITIVITY) - gyro_bias_z;

  // подавляем шум покоя: если скорость < 0.3 °/с — считаем нулём
  if (fabsf(rate_dps) < 0.3f) rate_dps = 0.0f;

  // 2. Интегрируем heading
  heading_deg += rate_dps * dt;

  // нормализуем в -180..180
  while (heading_deg >  180.0f) heading_deg -= 360.0f;
  while (heading_deg < -180.0f) heading_deg += 360.0f;

  // 3. Dead reckoning: средняя скорость колёс + heading → Δx, Δy
  float v = (speed_left_mps + speed_right_mps) * 0.5f;
  float rad = heading_deg * (PI / 180.0f);
  pos_x_m += v * cosf(rad) * dt;
  pos_y_m += v * sinf(rad) * dt;
}

void reset_imu(void) {
  heading_deg = 0.0f;
  pos_x_m = 0.0f;
  pos_y_m = 0.0f;
}

// ================= ПРОТОКОЛ: ПРИЁМ =================
#define RX_BUF_SIZE 256

uint8_t  rx_buf[RX_BUF_SIZE];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;

uint16_t rx_count(void) {
  return (rx_head >= rx_tail) ? (rx_head - rx_tail)
                              : (RX_BUF_SIZE + rx_head - rx_tail);
}

void uart_pump(void) {
  while (PiSerial.available()) {
    uint16_t next = (rx_head + 1) & (RX_BUF_SIZE - 1);
    if (next == rx_tail) break;
    rx_buf[rx_head] = (uint8_t)PiSerial.read();
    rx_head = next;
  }
}

bool is_it_data_packet(void) {
  if (rx_buf[rx_tail] != 0xAA) return false;
  if (rx_count() < 7) return false;
  return rx_buf[(rx_tail + 6) & (RX_BUF_SIZE - 1)] == 0xBB;
}

bool parse_and_pack(byte_packet* pkt) {
  bool found = false;
  while (rx_count() >= 7) {
    if (is_it_data_packet()) { found = true; break; }
    rx_tail = (rx_tail + 1) & (RX_BUF_SIZE - 1);
  }
  if (!found) return false;
  for (int i = 0; i < 5; i++)
    pkt->args[i] = rx_buf[(rx_tail + 1 + i) & (RX_BUF_SIZE - 1)];
  rx_tail = (rx_tail + 7) & (RX_BUF_SIZE - 1);
  return true;
}

bool is_crc_valid(byte_packet* p) {
  return (p->args[0] ^ p->args[1] ^ p->args[2] ^ p->args[3]) == p->args[4];
}

// ================= ДИСПЕТЧЕР =================
void execute_table(byte_packet* packet) {
  uint8_t cmd = packet->args[0];
  last_packet_received_time = millis();

  switch (cmd) {
    case CMD_EMERGENCY_STOP:
      target_speed_left  = 0.0f;
      target_speed_right = 0.0f;
      pid_left.Integral  = 0.0f;
      pid_right.Integral = 0.0f;
      break;

    case CMD_SET_SPEED:
      target_speed_left  = (float)(int8_t)packet->args[1];
      target_speed_right = (float)(int8_t)packet->args[2];
      break;

    case CMD_RESET_ODOMETER:
      odometer_left  = 0.0f;
      odometer_right = 0.0f;
      break;

    case CMD_TUNING_PID: {
      float new_val = packet->args[2] + (packet->args[3] / 100.0f);
      uint8_t coef  = packet->args[1];
      if (coef == 1) { pid_left.Kp = new_val; pid_right.Kp = new_val; }
      else if (coef == 2) {
        pid_left.Ki  = new_val; pid_left.Integral  = 0.0f;
        pid_right.Ki = new_val; pid_right.Integral = 0.0f;
      }
      else if (coef == 3) { pid_left.Kd = new_val; pid_right.Kd = new_val; }
      break;
    }

    case CMD_CONTROL_SERVOS: {
      uint8_t sid = packet->args[1];
      float angle = (float)packet->args[2];
      if (angle > 180.0f) angle = 180.0f;
      if (sid < 2) {
        target_servo_angles[sid] = angle;
        set_servo_angle(sid, angle);
      }
      break;
    }

    case CMD_RESET_IMU:
      reset_imu();
      break;

    default:
      break;
  }
}

// ================= БЕЗОПАСНОСТЬ =================
void check_connection_timeout(uint32_t now) {
  if ((now - last_packet_received_time) > COMMAND_TIMEOUT_MS) {
    target_speed_left  = 0.0f;
    target_speed_right = 0.0f;
    pid_left.Integral  = 0.0f;
    pid_right.Integral = 0.0f;
  }
}

// ================= ТЕЛЕМЕТРИЯ =================
#define MAX_RLE_PAIRS 32   // увеличено: 16 байт телеметрии → может быть больше пар

uint16_t rle_compression(const uint8_t* data, uint16_t len, rle_pair* out, uint16_t max_out) {
  if (len == 0) return 0;
  uint16_t j = 0;
  uint8_t cur = data[0];
  uint16_t cnt = 1;
  for (uint16_t i = 1; i < len; i++) {
    if (j >= max_out) return j;
    if (data[i] != data[i - 1]) {
      out[j].command = cur; out[j].counter = (uint8_t)cnt; j++;
      cur = data[i]; cnt = 1;
    } else {
      if (cnt == 255) {
        out[j].command = cur; out[j].counter = 255; j++;
        cnt = 0;
      }
      cnt++;
    }
  }
  if (j < max_out) { out[j].command = cur; out[j].counter = (uint8_t)cnt; j++; }
  return j;
}

void send_telemetry(raw_telemetry* t) {
  rle_pair pairs[MAX_RLE_PAIRS];
  uint16_t n = rle_compression((uint8_t*)t, sizeof(raw_telemetry), pairs, MAX_RLE_PAIRS);
  uint16_t len = n * sizeof(rle_pair);

  uint8_t* raw = (uint8_t*)pairs;
  uint8_t crc = (uint8_t)len;
  for (uint16_t i = 0; i < len; i++) crc ^= raw[i];

  PiSerial.write(0xCC);
  PiSerial.write((uint8_t)len);
  PiSerial.write(raw, len);
  PiSerial.write(crc);
  PiSerial.write(0xDD);
}

int8_t clamp_i8(float v) {
  if (v >  100.0f) v =  100.0f;
  if (v < -100.0f) v = -100.0f;
  return (int8_t)v;
}

int16_t clamp_i16(float v, float lo, float hi) {
  if (v > hi) v = hi;
  if (v < lo) v = lo;
  return (int16_t)v;
}

// ================= SETUP / LOOP =================
uint32_t last_locomotion_time = 0;
uint32_t last_telemetry_time  = 0;
uint32_t last_debug_time      = 0;

raw_telemetry telemetry = {};

void setup() {
  Serial.begin(115200);
  PiSerial.begin(115200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

  init_motors();
  init_servos();
  init_encoders();
  init_imu();           // калибровка ~1 с, ровер должен стоять!

  pid_init(&pid_left,  1.0f, 0.0f, 0.0f, 100.0f, -100.0f);
  pid_init(&pid_right, 1.0f, 0.0f, 0.0f, 100.0f, -100.0f);

  set_servo_angle(0, 90.0f);
  set_servo_angle(1, 90.0f);

  last_packet_received_time = millis();
  Serial.println("MOOLROVER ESP32: step3 (IMU) up");
}

void loop() {
  uint32_t now = millis();

  // I. приём
  uart_pump();
  byte_packet pkt;
  if (parse_and_pack(&pkt)) {
    if (is_crc_valid(&pkt)) execute_table(&pkt);
  }

  // II. безопасность
  check_connection_timeout(now);

  // III. контур управления + IMU, 20 мс
  if (now - last_locomotion_time >= 20) {
    float dt = (now - last_locomotion_time) / 1000.0f;
    last_locomotion_time = now;
    update_motors(dt);
    update_imu(dt);       // heading + dead reckoning
  }

  // IV. телеметрия, 100 мс
  if (now - last_telemetry_time >= 100) {
    last_telemetry_time = now;

    telemetry.status         = 0x01;
    telemetry.battery        = 100;
    telemetry.left_speed     = clamp_i8(speed_left_pct);
    telemetry.right_speed    = clamp_i8(speed_right_pct);
    telemetry.sonar_distance = 0;
    telemetry.heading_deg10  = clamp_i16(heading_deg * 10.0f, -1800.0f, 1800.0f);
    telemetry.pos_x_cm       = clamp_i16(pos_x_m * 100.0f, -32000.0f, 32000.0f);
    telemetry.pos_y_cm       = clamp_i16(pos_y_m * 100.0f, -32000.0f, 32000.0f);
    telemetry.imu_status     = imu_ok;

    send_telemetry(&telemetry);
  }

  // V. отладка, 500 мс
  if (now - last_debug_time >= 500) {
    last_debug_time = now;
    Serial.print("tgt L/R: "); Serial.print(target_speed_left, 0);
    Serial.print("/");         Serial.print(target_speed_right, 0);
    Serial.print(" | hdg: ");  Serial.print(heading_deg, 1);
    Serial.print("° | pos: "); Serial.print(pos_x_m, 2);
    Serial.print(",");         Serial.print(pos_y_m, 2);
    Serial.print("m | imu:");  Serial.println(imu_ok);
  }
}
