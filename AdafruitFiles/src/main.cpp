#include <Arduino.h>

#include "bluetooth.h"
#include "hardware_config.h"
#include "imu.h"
#include "motors.h"
#include "sensors.h"
#include "tasks.h"

float dt = 0.0025f;
bool flightSystemReady = false;

static bool createPinnedTask(TaskFunction_t taskCode,
                             const char *name,
                             uint32_t stackDepth,
                             UBaseType_t priority,
                             BaseType_t coreId) {
    const BaseType_t result =
        xTaskCreatePinnedToCore(taskCode, name, stackDepth, nullptr, priority, nullptr, coreId);
    if (result != pdPASS) {
        Serial.print("Task create failed: ");
        Serial.println(name);
        return false;
    }
    return true;
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);
    delay(200);
    Serial.println("SASE AQI Drone Metro ESP32-S3 boot");

    initBluetooth(BT_BAUD);
    initMotors();
    const bool imuOk = initIMU();
    const bool sensorsOk = initSensors();
    flightSystemReady = imuOk;
    if (!sensorsOk) {
        Serial.println("Warning: no healthy sensors at boot");
    }
    if (!flightSystemReady) {
        Serial.println("Flight disabled: critical IMU initialization failed");
        stopMotors();
    }

    // Pin the control loop away from slower I2C/range-sensor polling.
    const bool tasksOk =
        createPinnedTask(flightTask, "flightTask", 8192, 3, FLIGHT_CORE) &&
        createPinnedTask(sensorTask, "sensorTask", 8192, 2, SENSOR_CORE) &&
        createPinnedTask(telemetryTask, "telemetryTask", 4096, 1, SENSOR_CORE);
    if (!tasksOk) {
        flightSystemReady = false;
        stopMotors();
        Serial.println("Flight disabled: task creation failed");
    }

    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
