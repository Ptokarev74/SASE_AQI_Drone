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
    uint32_t lastHeartbeatMs = 0;

    for (;;) {
        const uint32_t loopStartUs = micros();
        dt = (float)(loopStartUs - previousMicros) / 1000000.0f;
        previousMicros = loopStartUs;
        // Clamp abnormal timing so one delayed loop cannot spike the PID terms.
        if (dt <= 0.0f || dt > 0.1f) {
            dt = (float)FLIGHT_TASK_PERIOD_US / 1000000.0f;
        }

        // Keep the flight loop bounded: slow sensors run in sensorTask.
        bluetoothPoll();
        if (flightSystemReady) {
            readIMU();
            if (!imuReady) {
                flightSystemReady = false;
                Serial.println("Flight disabled: IMU became unavailable");
            }
        }

        if (flightSystemReady) {
            updateState();
            calculatePID();
            mixMotors();
        } else {
            setState(IDLE);
            stopMotors();
        }
        publishFlightTelemetrySnapshot();

        const uint32_t nowMs = millis();

        if (ENABLE_DEBUG_HEARTBEAT && nowMs - lastHeartbeatMs >= 2000) {
            lastHeartbeatMs = nowMs;
            Serial.println("Flight task heartbeat");
        }

        // Pace the loop to the configured control period, yielding on overrun.
        const uint32_t elapsedUs = micros() - loopStartUs;
        if (elapsedUs < FLIGHT_TASK_PERIOD_US) {
            delayMicroseconds(FLIGHT_TASK_PERIOD_US - elapsedUs);
        } else {
            taskYIELD();
        }
    }
}

void telemetryTask(void *parameter) {
    (void)parameter;
    for (;;) {
        sendTelemetry();
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}
