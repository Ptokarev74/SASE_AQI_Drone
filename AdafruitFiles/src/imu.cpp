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

bool initIMU() {
    pinMode(BNO08X_RESET_PIN, OUTPUT);
    digitalWrite(BNO08X_RESET_PIN, LOW);
    delay(100);
    digitalWrite(BNO08X_RESET_PIN, HIGH);
    delay(100);

    if (!bno08x.begin_SPI(BNO08X_CS_PIN, BNO08X_INT_PIN, &SPI)) {
        Serial.println("BNO08x init failed");
        return false;
    }

    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 2500)) {
        Serial.println("BNO08x rotation vector report failed");
        return false;
    }

    float sumRoll = 0.0f;
    float sumPitch = 0.0f;
    float sumYaw = 0.0f;
    int samples = 0;
    const uint32_t startMs = millis();

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
            sumYaw += yaw;
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
    yawOffset = sumYaw / (float)samples;

    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 2500);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 5000);

    sh2_SensorValue_t discarded;
    while (bno08x.getSensorEvent(&discarded)) {
    }

    Serial.println("BNO08x ready");
    return true;
}

void readIMU() {
    int eventsRead = 0;
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
                yawIMU = rawYaw - yawOffset;
                break;
            }
            case SH2_GYROSCOPE_CALIBRATED:
                gyroX = sensorValue.un.gyroscope.x * RAD_TO_DEG;
                gyroY = sensorValue.un.gyroscope.y * RAD_TO_DEG;
                gyroZ = sensorValue.un.gyroscope.z * RAD_TO_DEG;
                break;
            default:
                break;
        }
    }
}
