#include <Arduino.h>
#include <SparkFun_LPS28DFW_Arduino_Library.h>
#include <Wire.h>
#include <math.h>

#include "barometer.h"

static LPS28DFW pressureSensor;
static bool barometerReady = false;
static float pressureBuffer[16] = {0.0f};
static size_t pressureBufferIndex = 0;
static size_t pressureSamples = 0;
static float pressureRunningSum = 0.0f;
static float latestPressureHpa = 0.0f;
static float latestAltitudeM = 0.0f;
static float baselinePressureHpa = 0.0f;
static bool baselinePressureSet = false;
static bool latestBarometerOk = false;
static uint32_t lastBarometerUpdateMs = 0;
static constexpr uint32_t BAROMETER_STALE_MS = 250;

bool initBarometer() {
    if (pressureSensor.begin(LPS28DFW_I2C_ADDRESS_DEFAULT, Wire) != LPS28DFW_OK) {
        Serial.println("LPS28DFW not found");
        barometerReady = false;
        return false;
    }

    lps28dfw_md_t modeConfig;
    modeConfig.fs = LPS28DFW_1260hPa;
    modeConfig.odr = LPS28DFW_200Hz;
    modeConfig.avg = LPS28DFW_4_AVG;
    modeConfig.lpf = LPS28DFW_LPF_DISABLE;
    pressureSensor.setModeConfig(&modeConfig);

    Serial.println("LPS28DFW ready");
    barometerReady = true;
    return true;
}

bool updateBarometer() {
    if (!barometerReady) {
        latestBarometerOk = false;
        return false;
    }

    if (pressureSensor.getSensorData() != LPS28DFW_OK) {
        if (lastBarometerUpdateMs == 0 || millis() - lastBarometerUpdateMs > BAROMETER_STALE_MS) {
            latestBarometerOk = false;
        }
        return false;
    }

    latestPressureHpa = pressureSensor.data.pressure.hpa;
    if (latestPressureHpa <= 0.0f) {
        latestBarometerOk = false;
        return false;
    }

    // Small moving average smooths pressure noise without adding much latency.
    pressureRunningSum -= pressureBuffer[pressureBufferIndex];
    pressureBuffer[pressureBufferIndex] = latestPressureHpa;
    pressureRunningSum += latestPressureHpa;
    pressureBufferIndex = (pressureBufferIndex + 1) % 16;
    if (pressureSamples < 16) {
        pressureSamples++;
    }

    const float filteredPressure = pressureRunningSum / (float)pressureSamples;
    if (!baselinePressureSet) {
        baselinePressureHpa = filteredPressure;
        baselinePressureSet = true;
    }

    // Report altitude relative to startup instead of assuming sea-level pressure.
    latestAltitudeM = 44330.0f * (1.0f - powf(filteredPressure / baselinePressureHpa, 0.1903f));
    latestBarometerOk = true;
    lastBarometerUpdateMs = millis();
    return true;
}

bool isBarometerReady() {
    return barometerReady;
}

bool isBarometerOk() {
    return barometerReady &&
           latestBarometerOk &&
           lastBarometerUpdateMs != 0 &&
           millis() - lastBarometerUpdateMs <= BAROMETER_STALE_MS;
}

float getLatestPressureHpa() {
    return latestPressureHpa;
}

float getLatestAltitudeM() {
    return latestAltitudeM;
}
