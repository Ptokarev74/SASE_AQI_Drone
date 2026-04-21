#include <Arduino.h>

#include "imu.h"
#include "main.h"
#include "motors.h"
#include "pid.h"

float kpRoll = 1.5f;
float kiRoll = 0.5f;
float kdRoll = 0.05f;
float kpPitch = 1.5f;
float kiPitch = 0.5f;
float kdPitch = 0.05f;
float kpYaw = 2.0f;
float kiYaw = 0.0f;
float kdYaw = 0.05f;
float integralLimit = 200.0f;

float rollPid = 0.0f;
float pitchPid = 0.0f;
float yawPid = 0.0f;
float integralRoll = 0.0f;
float integralPitch = 0.0f;
float integralYaw = 0.0f;

float rollDesired = 0.0f;
float pitchDesired = 0.0f;
float yawDesired = 0.0f;

void calculatePID() {
    const float errorRoll = rollDesired - rollIMU;
    if (throttleDesired >= 50) {
        integralRoll += errorRoll * dt;
    }
    integralRoll = constrain(integralRoll, -integralLimit, integralLimit);
    rollPid = (kpRoll * errorRoll) + (kiRoll * integralRoll) - (kdRoll * gyroX);

    const float errorPitch = pitchDesired - pitchIMU;
    if (throttleDesired >= 50) {
        integralPitch += errorPitch * dt;
    }
    integralPitch = constrain(integralPitch, -integralLimit, integralLimit);
    pitchPid = (kpPitch * errorPitch) + (kiPitch * integralPitch) - (kdPitch * gyroY);

    const float errorYaw = yawDesired - yawIMU;
    if (throttleDesired >= 50) {
        integralYaw += errorYaw * dt;
    }
    integralYaw = constrain(integralYaw, -integralLimit, integralLimit);
    yawPid = (kpYaw * errorYaw) + (kiYaw * integralYaw) - (kdYaw * gyroZ);
}
