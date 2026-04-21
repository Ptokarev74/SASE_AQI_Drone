#include <Arduino.h>
#include <Wire.h>

#include "barometer.h"
#include "hardware_config.h"
#include "sensors.h"
#include "tof.h"
#include "ultrasonic.h"

static portMUX_TYPE sensorSnapshotMux = portMUX_INITIALIZER_UNLOCKED;
static SensorSnapshot sharedSensorSnapshot;

SensorSnapshot getSensorSnapshot() {
    SensorSnapshot snapshot;
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
    snapshot.barometerOk = isBarometerReady() && snapshot.pressureHpa > 0.0f;
    snapshot.ultrasonicOk = isUltrasonicOk();
    snapshot.tofOk = isTofOk();
    snapshot.updatedMs = millis();

    portENTER_CRITICAL(&sensorSnapshotMux);
    sharedSensorSnapshot = snapshot;
    portEXIT_CRITICAL(&sensorSnapshotMux);
}

void initSensors() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);
    initBarometer();
    initTofSensor();
    initUltrasonic();
    publishSensorSnapshot();
}

void sensorTask(void *parameter) {
    (void)parameter;
    uint32_t lastTofMs = 0;
    uint32_t lastUltrasonicTriggerMs = 0;

    for (;;) {
        const uint32_t nowMs = millis();
        updateBarometer();

        if (nowMs - lastTofMs >= 50) {
            lastTofMs = nowMs;
            updateTofSensor();
        }

        if (nowMs - lastUltrasonicTriggerMs >= 60) {
            lastUltrasonicTriggerMs = nowMs;
            startUltrasonicMeasurement();
        }
        updateUltrasonic();
        publishSensorSnapshot();

        vTaskDelay(pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}
