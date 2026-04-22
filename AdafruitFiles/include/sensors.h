#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

// Latest cross-task sensor values. Distances are cm, altitude is meters, and
// pressure is hPa.
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

bool initSensors();
bool areSensorsReady();
SensorSnapshot getSensorSnapshot();
void publishSensorSnapshot();
void sensorTask(void *parameter);

#endif
