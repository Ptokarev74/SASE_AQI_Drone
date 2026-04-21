#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

struct SensorSnapshot {
    float altitudeM = 0.0f;
    float pressureHpa = 0.0f;
    float ultrasonicCm = 9999.0f;
    float tofCm = 9999.0f;
    bool barometerOk = false;
    bool ultrasonicOk = false;
    bool tofOk = false;
    uint32_t updatedMs = 0;
};

void initSensors();
SensorSnapshot getSensorSnapshot();
void publishSensorSnapshot();
void sensorTask(void *parameter);

#endif
