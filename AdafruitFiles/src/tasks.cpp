#include <Arduino.h>

#include "bluetooth.h"
#include "hardware_config.h"
#include "imu.h"
#include "main.h"
#include "motors.h"
#include "pid.h"
#include "states.h"
#include "telemetry.h"

void flightTask(void *parameter) {
    (void)parameter;
    uint32_t previousMicros = micros();
    uint32_t lastTelemetryMs = 0;
    uint32_t lastHeartbeatMs = 0;

    for (;;) {
        const uint32_t loopStartUs = micros();
        dt = (float)(loopStartUs - previousMicros) / 1000000.0f;
        previousMicros = loopStartUs;
        if (dt <= 0.0f || dt > 0.1f) {
            dt = (float)FLIGHT_TASK_PERIOD_US / 1000000.0f;
        }

        bluetoothPoll();
        readIMU();
        updateState();
        calculatePID();
        mixMotors();

        const uint32_t nowMs = millis();
        if (nowMs - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
            lastTelemetryMs = nowMs;
            sendTelemetry();
        }

        if (ENABLE_DEBUG_HEARTBEAT && nowMs - lastHeartbeatMs >= 2000) {
            lastHeartbeatMs = nowMs;
            Serial.println("Flight task heartbeat");
        }

        const uint32_t elapsedUs = micros() - loopStartUs;
        if (elapsedUs < FLIGHT_TASK_PERIOD_US) {
            delayMicroseconds(FLIGHT_TASK_PERIOD_US - elapsedUs);
        } else {
            taskYIELD();
        }
    }
}
