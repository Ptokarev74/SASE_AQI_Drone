#include <Arduino.h>
#include <ArduinoJson.h>

#include "bluetooth.h"
#include "imu.h"
#include "main.h"
#include "motors.h"
#include "pid.h"
#include "sensors.h"
#include "states.h"
#include "telemetry.h"

static portMUX_TYPE flightTelemetryMux = portMUX_INITIALIZER_UNLOCKED;
static FlightTelemetrySnapshot sharedFlightTelemetrySnapshot;

void publishFlightTelemetrySnapshot() {
    FlightTelemetrySnapshot snapshot;
    snapshot.status = stateName();
    snapshot.pitch = pitchIMU;
    snapshot.roll = rollIMU;
    snapshot.yaw = yawIMU;
    snapshot.targetPitch = pitchDesired;
    snapshot.targetRoll = rollDesired;
    snapshot.throttle = throttleDesired;
    snapshot.armed = droneArmed;
    snapshot.imuOk = imuReady;
    snapshot.flightReady = flightSystemReady;
    snapshot.sensorsReady = areSensorsReady();
    snapshot.bluetoothConnected = bluetoothConnected;
    snapshot.updatedMs = millis();

    portENTER_CRITICAL(&flightTelemetryMux);
    sharedFlightTelemetrySnapshot = snapshot;
    portEXIT_CRITICAL(&flightTelemetryMux);
}

FlightTelemetrySnapshot getFlightTelemetrySnapshot() {
    FlightTelemetrySnapshot snapshot;
    portENTER_CRITICAL(&flightTelemetryMux);
    snapshot = sharedFlightTelemetrySnapshot;
    portEXIT_CRITICAL(&flightTelemetryMux);
    return snapshot;
}

void sendTelemetry() {
    const SensorSnapshot sensors = getSensorSnapshot();
    const FlightTelemetrySnapshot flight = getFlightTelemetrySnapshot();
    JsonDocument doc;
    doc["status"] = flight.status;
    doc["pitch"] = flight.pitch;
    doc["roll"] = flight.roll;
    doc["yaw"] = flight.yaw;
    doc["target_pitch"] = flight.targetPitch;
    doc["target_roll"] = flight.targetRoll;
    doc["throttle"] = flight.throttle;
    doc["armed"] = flight.armed;
    // Placeholder until battery sensing hardware is added.
    doc["battery"] = 100;
    doc["altitude_m"] = sensors.altitudeM;
    doc["pressure_hpa"] = sensors.pressureHpa;
    doc["ultrasonic_cm"] = sensors.ultrasonicCm;
    doc["tof_cm"] = sensors.tofCm;
    doc["sensor_ok"] = sensors.barometerOk || sensors.ultrasonicOk || sensors.tofOk;
    doc["barometer_ok"] = sensors.barometerOk;
    doc["ultrasonic_ok"] = sensors.ultrasonicOk;
    doc["tof_ok"] = sensors.tofOk;
    doc["imu_ok"] = flight.imuOk;
    doc["flight_ready"] = flight.flightReady;
    doc["sensors_ready"] = flight.sensorsReady;
    doc["sensor_age_ms"] = millis() - sensors.updatedMs;
    doc["flight_age_ms"] = millis() - flight.updatedMs;
    doc["bt_connected"] = flight.bluetoothConnected;
    bluetoothSendJson(doc);
}
