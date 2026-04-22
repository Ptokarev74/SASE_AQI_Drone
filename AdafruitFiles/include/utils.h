#ifndef UTILS_H
#define UTILS_H

void quaternionToEuler(float r, float i, float j, float k, float &roll, float &pitch, float &yaw);
float normalizeAngleDegrees(float angle);

#endif
