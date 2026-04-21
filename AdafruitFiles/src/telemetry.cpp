#include <Arduino.h>
#include <ArduinoJson.h>

#include "bluetooth.h"
#include "imu.h"
#include "motors.h"
#include "pid.h"
#include "sensors.h"
#include "states.h"
#include "telemetry.h"

void sendTelemetry() {
    const SensorSnapshot sensors = getSensorSnapshot();
    JsonDocument doc;
    doc["status"] = stateName();
    doc["pitch"] = pitchIMU;
    doc["roll"] = rollIMU;
    doc["yaw"] = yawIMU;
    doc["target_pitch"] = pitchDesired;
    doc["target_roll"] = rollDesired;
    doc["throttle"] = throttleDesired;
    doc["battery"] = 100;
    doc["altitude_m"] = sensors.altitudeM;
    doc["pressure_hpa"] = sensors.pressureHpa;
    doc["ultrasonic_cm"] = sensors.ultrasonicCm;
    doc["tof_cm"] = sensors.tofCm;
    doc["sensor_ok"] = sensors.barometerOk || sensors.ultrasonicOk || sensors.tofOk;
    doc["bt_connected"] = bluetoothConnected;
    bluetoothSendJson(doc);
}
