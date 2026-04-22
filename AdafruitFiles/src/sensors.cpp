#include <Arduino.h>
#include <Wire.h>

#include "barometer.h"
#include "hardware_config.h"
#include "main.h"
#include "sensors.h"
#include "tof.h"
#include "ultrasonic.h"

static portMUX_TYPE sensorSnapshotMux = portMUX_INITIALIZER_UNLOCKED;
static SensorSnapshot sharedSensorSnapshot;

static bool hasHealthySensor(const SensorSnapshot &snapshot) {
    return snapshot.barometerOk || snapshot.ultrasonicOk || snapshot.tofOk;
}

SensorSnapshot getSensorSnapshot() {
    SensorSnapshot snapshot;
    // Copy the complete snapshot while interrupts/tasks cannot modify it.
    portENTER_CRITICAL(&sensorSnapshotMux);
    snapshot = sharedSensorSnapshot;
    portEXIT_CRITICAL(&sensorSnapshotMux);
    return snapshot;
}

void publishSensorSnapshot() {
    SensorSnapshot snapshot;
    snapshot.altitudeM = getLatestAltitudeM();
    snapshot.pressureHpa = getLatestPressureHpa();
    snapshot.ultrasonicCm = getLatestUltrasonicCm();
    snapshot.tofCm = getLatestTofCm();
    snapshot.barometerOk = isBarometerOk();
    snapshot.ultrasonicOk = isUltrasonicOk();
    snapshot.tofOk = isTofOk();
    snapshot.updatedMs = millis();

    // Publish one coherent set of readings for the flight and telemetry code.
    portENTER_CRITICAL(&sensorSnapshotMux);
    sharedSensorSnapshot = snapshot;
    portEXIT_CRITICAL(&sensorSnapshotMux);
}

bool areSensorsReady() {
    return hasHealthySensor(getSensorSnapshot());
}

bool initSensors() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);
    initBarometer();
    initTofSensor();
    initUltrasonic();
    updateBarometer();
    updateTofSensor();
    publishSensorSnapshot();
    return areSensorsReady();
}

void sensorTask(void *parameter) {
    (void)parameter;
    uint32_t lastTofMs = 0;
    uint32_t lastUltrasonicTriggerMs = 0;

    for (;;) {
        const uint32_t nowMs = millis();
        // Barometer is cheap enough to poll on every sensor-task tick.
        updateBarometer();

        if (nowMs - lastTofMs >= 50) {
            lastTofMs = nowMs;
            updateTofSensor();
        }

        // HC-SR04 measurements are triggered asynchronously and completed by
        // the echo interrupt/update pair.
        if (nowMs - lastUltrasonicTriggerMs >= 60) {
            lastUltrasonicTriggerMs = nowMs;
            startUltrasonicMeasurement();
        }
        updateUltrasonic();
        publishSensorSnapshot();

        vTaskDelay(pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}
