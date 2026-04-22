#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <SPI.h>

#include "hardware_config.h"
#include "imu.h"
#include "utils.h"

Adafruit_BNO08x bno08x(BNO08X_RESET_PIN);
sh2_SensorValue_t sensorValue;

float rollIMU = 0.0f;
float pitchIMU = 0.0f;
float yawIMU = 0.0f;
float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;
float rollOffset = 0.0f;
float pitchOffset = 0.0f;
float yawOffset = 0.0f;
bool imuReady = false;

static constexpr uint32_t IMU_FAST_REPORT_INTERVAL_US = 2500;
static constexpr uint32_t IMU_ACCEL_REPORT_INTERVAL_US = 5000;
static constexpr uint32_t IMU_STALE_TIMEOUT_MS = 100;
static uint32_t lastRotationReportMs = 0;
static uint32_t lastGyroReportMs = 0;

static bool enableRequiredReports() {
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_FAST_REPORT_INTERVAL_US)) {
        Serial.println("BNO08x rotation vector report failed");
        return false;
    }
    if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, IMU_FAST_REPORT_INTERVAL_US)) {
        Serial.println("BNO08x gyroscope report failed");
        return false;
    }
    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, IMU_ACCEL_REPORT_INTERVAL_US)) {
        Serial.println("BNO08x linear acceleration report failed");
    }
    return true;
}

bool initIMU() {
    imuReady = false;
    lastRotationReportMs = 0;
    lastGyroReportMs = 0;

    // Hardware reset helps the BNO08x start reliably after firmware uploads.
    pinMode(BNO08X_RESET_PIN, OUTPUT);
    digitalWrite(BNO08X_RESET_PIN, LOW);
    delay(100);
    digitalWrite(BNO08X_RESET_PIN, HIGH);
    delay(100);

    if (!bno08x.begin_SPI(BNO08X_CS_PIN, BNO08X_INT_PIN, &SPI)) {
        Serial.println("BNO08x init failed");
        return false;
    }

    if (!enableRequiredReports()) {
        return false;
    }

    float sumRoll = 0.0f;
    float sumPitch = 0.0f;
    float sumYawSin = 0.0f;
    float sumYawCos = 0.0f;
    int samples = 0;
    const uint32_t startMs = millis();

    // Average the startup attitude and treat it as the level reference.
    while (samples < 20 && millis() - startMs < 3000) {
        if (bno08x.getSensorEvent(&sensorValue) &&
            sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            float roll = 0.0f;
            float pitch = 0.0f;
            float yaw = 0.0f;
            quaternionToEuler(
                sensorValue.un.gameRotationVector.real,
                sensorValue.un.gameRotationVector.i,
                sensorValue.un.gameRotationVector.j,
                sensorValue.un.gameRotationVector.k,
                roll,
                pitch,
                yaw);
            sumRoll += roll;
            sumPitch += pitch;
            const float yawRad = yaw * DEG_TO_RAD;
            sumYawSin += sinf(yawRad);
            sumYawCos += cosf(yawRad);
            samples++;
        }
        delay(5);
    }

    if (samples == 0) {
        Serial.println("BNO08x calibration timed out");
        return false;
    }

    rollOffset = sumRoll / (float)samples;
    pitchOffset = sumPitch / (float)samples;
    yawOffset = atan2f(sumYawSin, sumYawCos) * RAD_TO_DEG;

    // Clear queued calibration samples before the flight loop starts.
    sh2_SensorValue_t discarded;
    while (bno08x.getSensorEvent(&discarded)) {
    }

    Serial.println("BNO08x ready");
    lastRotationReportMs = millis();
    lastGyroReportMs = lastRotationReportMs;
    imuReady = true;
    return true;
}

void readIMU() {
    if (!imuReady) {
        return;
    }

    if (bno08x.wasReset()) {
        Serial.println("BNO08x reset detected; flight disabled until recalibration");
        imuReady = false;
        return;
    }

    int eventsRead = 0;
    // Bound per-loop IMU work so stale queued events cannot starve control.
    while (eventsRead < 8 && bno08x.getSensorEvent(&sensorValue)) {
        eventsRead++;
        switch (sensorValue.sensorId) {
            case SH2_GAME_ROTATION_VECTOR: {
                float rawRoll = 0.0f;
                float rawPitch = 0.0f;
                float rawYaw = 0.0f;
                quaternionToEuler(
                    sensorValue.un.gameRotationVector.real,
                    sensorValue.un.gameRotationVector.i,
                    sensorValue.un.gameRotationVector.j,
                    sensorValue.un.gameRotationVector.k,
                    rawRoll,
                    rawPitch,
                    rawYaw);
                rollIMU = rawRoll - rollOffset;
                pitchIMU = rawPitch - pitchOffset;
                yawIMU = normalizeAngleDegrees(rawYaw - yawOffset);
                lastRotationReportMs = millis();
                break;
            }
            case SH2_GYROSCOPE_CALIBRATED:
                gyroX = sensorValue.un.gyroscope.x * RAD_TO_DEG;
                gyroY = sensorValue.un.gyroscope.y * RAD_TO_DEG;
                gyroZ = sensorValue.un.gyroscope.z * RAD_TO_DEG;
                lastGyroReportMs = millis();
                break;
            default:
                break;
        }
    }

    const uint32_t checkMs = millis();
    if ((lastRotationReportMs == 0 || checkMs - lastRotationReportMs > IMU_STALE_TIMEOUT_MS) ||
        (lastGyroReportMs == 0 || checkMs - lastGyroReportMs > IMU_STALE_TIMEOUT_MS)) {
        Serial.println("BNO08x reports stale");
        imuReady = false;
    }
}
