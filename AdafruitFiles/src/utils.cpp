#include <Arduino.h>
#include <math.h>

#include "utils.h"

void quaternionToEuler(float r, float i, float j, float k, float &roll, float &pitch, float &yaw) {
    const float sinrCosp = 2.0f * (r * i + j * k);
    const float cosrCosp = 1.0f - 2.0f * (i * i + j * j);
    roll = atan2f(sinrCosp, cosrCosp) * RAD_TO_DEG;

    const float sinp = 2.0f * (r * j - k * i);
    pitch = (fabsf(sinp) >= 1.0f) ? copysignf(90.0f, sinp) : asinf(sinp) * RAD_TO_DEG;

    const float sinyCosp = 2.0f * (r * k + i * j);
    const float cosyCosp = 1.0f - 2.0f * (j * j + k * k);
    yaw = atan2f(sinyCosp, cosyCosp) * RAD_TO_DEG;
}
