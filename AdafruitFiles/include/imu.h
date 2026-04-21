#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO08x.h>

extern Adafruit_BNO08x bno08x;
extern sh2_SensorValue_t sensorValue;

extern float rollIMU;
extern float pitchIMU;
extern float yawIMU;
extern float gyroX;
extern float gyroY;
extern float gyroZ;
extern float rollOffset;
extern float pitchOffset;
extern float yawOffset;

bool initIMU();
void readIMU();

#endif
