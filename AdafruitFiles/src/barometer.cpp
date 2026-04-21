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
    if (!barometerReady || pressureSensor.getSensorData() != LPS28DFW_OK) {
        return false;
    }

    latestPressureHpa = pressureSensor.data.pressure.hpa;
    pressureRunningSum -= pressureBuffer[pressureBufferIndex];
    pressureBuffer[pressureBufferIndex] = latestPressureHpa;
    pressureRunningSum += latestPressureHpa;
    pressureBufferIndex = (pressureBufferIndex + 1) % 16;
    if (pressureSamples < 16) {
        pressureSamples++;
    }

    const float filteredPressure = pressureRunningSum / (float)pressureSamples;
    const float seaLevelHpa = 1013.25f;
    latestAltitudeM = 44330.0f * (1.0f - powf(filteredPressure / seaLevelHpa, 0.1903f));
    return true;
}

bool isBarometerReady() {
    return barometerReady;
}

float getLatestPressureHpa() {
    return latestPressureHpa;
}

float getLatestAltitudeM() {
    return latestAltitudeM;
}
