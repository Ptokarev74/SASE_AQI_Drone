#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include <Arduino.h>

#ifndef D2
#define D2 2
#endif
#ifndef D3
#define D3 3
#endif
#ifndef D4
#define D4 4
#endif
#ifndef D5
#define D5 5
#endif
#ifndef D8
#define D8 8
#endif
#ifndef D9
#define D9 9
#endif
#ifndef D10
#define D10 10
#endif
#ifndef A0
#define A0 14
#endif
#ifndef A1
#define A1 15
#endif

// Update this file after the final Metro ESP32-S3 wiring is locked.
static constexpr int MOTOR_FRONT_RIGHT_PIN = D2;
static constexpr int MOTOR_BACK_RIGHT_PIN = D3;
static constexpr int MOTOR_BACK_LEFT_PIN = D4;
static constexpr int MOTOR_FRONT_LEFT_PIN = D5;

static constexpr int BNO08X_CS_PIN = D10;
static constexpr int BNO08X_INT_PIN = D9;
static constexpr int BNO08X_RESET_PIN = D8;

static constexpr int BT_RX_PIN = RX;
static constexpr int BT_TX_PIN = TX;
static constexpr unsigned long BT_BAUD = 9600;

static constexpr int I2C_SDA_PIN = SDA;
static constexpr int I2C_SCL_PIN = SCL;
static constexpr uint32_t I2C_CLOCK_HZ = 400000;

// HC-SR04 echo must be level shifted to 3.3 V before entering the ESP32-S3.
static constexpr int ULTRASONIC_TRIG_PIN = A0;
static constexpr int ULTRASONIC_ECHO_PIN = A1;

static constexpr uint8_t TOF_I2C_ADDR = 0x29;

static constexpr int ESC_PWM_HZ = 300;
static constexpr int ESC_PWM_BITS = 12;
static constexpr int PWM_OFF_US = 1000;
static constexpr int PWM_MAX_US = 2000;
static constexpr int THROTTLE_MAX_OFFSET_US = PWM_MAX_US - PWM_OFF_US;

static constexpr int FLIGHT_CORE = 1;
static constexpr int SENSOR_CORE = 0;
static constexpr uint32_t FLIGHT_TASK_PERIOD_US = 2500;
static constexpr uint32_t SENSOR_TASK_PERIOD_MS = 10;
static constexpr uint32_t TELEMETRY_PERIOD_MS = 500;
static constexpr uint32_t BLUETOOTH_TIMEOUT_MS = 5000;

static constexpr bool ENABLE_DEBUG_HEARTBEAT = false;

#endif
