#include <Arduino.h>

#include "hardware_config.h"
#include "imu.h"
#include "main.h"
#include "motors.h"
#include "pid.h"
#include "utils.h"

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
    if (throttleDesired < PID_ACTIVE_THROTTLE_OFFSET_US) {
        integralRoll = 0.0f;
        integralPitch = 0.0f;
        integralYaw = 0.0f;
        rollPid = 0.0f;
        pitchPid = 0.0f;
        yawPid = 0.0f;
        return;
    }

    const float errorRoll = rollDesired - rollIMU;
    integralRoll += errorRoll * dt;
    integralRoll = constrain(integralRoll, -integralLimit, integralLimit);
    // Derivative uses gyro rate directly to avoid differentiating noisy angles.
    rollPid = (kpRoll * errorRoll) + (kiRoll * integralRoll) - (kdRoll * gyroX);

    const float errorPitch = pitchDesired - pitchIMU;
    integralPitch += errorPitch * dt;
    integralPitch = constrain(integralPitch, -integralLimit, integralLimit);
    pitchPid = (kpPitch * errorPitch) + (kiPitch * integralPitch) - (kdPitch * gyroY);

    const float errorYaw = normalizeAngleDegrees(yawDesired - yawIMU);
    integralYaw += errorYaw * dt;
    integralYaw = constrain(integralYaw, -integralLimit, integralLimit);
    yawPid = (kpYaw * errorYaw) + (kiYaw * integralYaw) - (kdYaw * gyroZ);
}
