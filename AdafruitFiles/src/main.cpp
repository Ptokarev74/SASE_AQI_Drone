#include <Arduino.h>

#include "bluetooth.h"
#include "hardware_config.h"
#include "imu.h"
#include "motors.h"
#include "sensors.h"
#include "tasks.h"

float dt = 0.0025f;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);
    delay(200);
    Serial.println("SASE AQI Drone Metro ESP32-S3 boot");

    initBluetooth(BT_BAUD);
    initMotors();
    initIMU();
    initSensors();

    xTaskCreatePinnedToCore(flightTask, "flightTask", 8192, nullptr, 3, nullptr, FLIGHT_CORE);
    xTaskCreatePinnedToCore(sensorTask, "sensorTask", 8192, nullptr, 2, nullptr, SENSOR_CORE);

    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
