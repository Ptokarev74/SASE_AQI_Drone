#include <Arduino.h>
#include <Adafruit_VL53L0X.h>
#include <Wire.h>

#include "hardware_config.h"
#include "tof.h"

static Adafruit_VL53L0X tofSensor;
static bool tofReady = false;
static bool latestTofOk = false;
static float latestTofCm = 9999.0f;
static constexpr uint8_t TOF_RANGE_STATUS_VALID = 0;
static constexpr float TOF_MIN_VALID_CM = 1.0f;
static constexpr float TOF_MAX_VALID_CM = 400.0f;

bool initTofSensor() {
    if (!tofSensor.begin(TOF_I2C_ADDR, false, &Wire)) {
        Serial.println("VL53L0X not found");
        tofReady = false;
        return false;
    }

    Serial.println("VL53L0X ready");
    tofReady = true;
    return true;
}

void updateTofSensor() {
    latestTofOk = false;
    if (!tofReady) {
        return;
    }

    VL53L0X_RangingMeasurementData_t measure;
    tofSensor.rangingTest(&measure, false);
    const float distanceCm = (float)measure.RangeMilliMeter / 10.0f;
    if (measure.RangeStatus == TOF_RANGE_STATUS_VALID &&
        distanceCm >= TOF_MIN_VALID_CM &&
        distanceCm <= TOF_MAX_VALID_CM) {
        latestTofCm = distanceCm;
        latestTofOk = true;
    } else {
        latestTofCm = 9999.0f;
    }
}

bool isTofReady() {
    return tofReady;
}

bool isTofOk() {
    return latestTofOk;
}

float getLatestTofCm() {
    return latestTofCm;
}
