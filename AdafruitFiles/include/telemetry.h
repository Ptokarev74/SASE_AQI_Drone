#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

struct FlightTelemetrySnapshot {
    const char *status = "idle";
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    float targetPitch = 0.0f;
    float targetRoll = 0.0f;
    int throttle = 0;
    bool armed = false;
    bool imuOk = false;
    bool flightReady = false;
    bool sensorsReady = false;
    bool bluetoothConnected = false;
    uint32_t updatedMs = 0;
};

void publishFlightTelemetrySnapshot();
FlightTelemetrySnapshot getFlightTelemetrySnapshot();
void sendTelemetry();

#endif
