#ifndef ROVER_TYPES_H
#define ROVER_TYPES_H

#include <stdint.h>

/*
  Все структуры вынесены сюда НЕ для красоты.
  Arduino IDE сама генерирует прототипы функций и вставляет их сразу после
  последнего #include. Если typedef лежит в .ino ниже — прототип с этим типом
  окажется выше определения типа, и компилятор упадёт.
  Через заголовок типы попадают в файл раньше прототипов.
*/

// --- PID ---
typedef struct {
  float Kp, Ki, Kd;
  float Integral;
  float previous_error;
  float max_output;
  float min_output;
} pid_controller;

// --- Кадр приёма: 0xAA | cmd | a1 | a2 | a3 | crc | 0xBB ---
typedef struct {
  uint8_t args[5];
} byte_packet;

// --- Телеметрия (16 байт до сжатия) ---
//     Расширена: heading, pos_x, pos_y, imu_status.
//     ОБЯЗАТЕЛЬНО обновить TELEM_SIZE на Python-стороне (esp32_link.py).
typedef struct __attribute__((packed)) {
  uint8_t status;           // 0:  статус робота
  uint8_t battery;          // 1:  напряжение * 10 (114 = 11.4 В)
  int8_t  left_speed;       // 2:  ПРОЦЕНТЫ (-100..100)
  int8_t  right_speed;      // 3:  ПРОЦЕНТЫ
  uint8_t sonar_distance;   // 4:  см
  int16_t heading_deg10;    // 5-6:  курс × 10 (±1800 = ±180.0°), от старта
  int16_t pos_x_cm;         // 7-8:  x в см от старта
  int16_t pos_y_cm;         // 9-10: y в см от старта
  uint8_t imu_status;       // 11: 0=нет IMU, 1=ок, 2=ошибка
  uint8_t reserved[4];      // 12-15: запас
} raw_telemetry;

// --- Пара RLE ---
typedef struct {
  uint8_t command;
  uint8_t counter;
} rle_pair;

#endif // ROVER_TYPES_H
