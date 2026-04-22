# SASE AQI Drone Context

This repo contains several generations of AQI drone work. For the current firmware work, treat `AdafruitFiles/` as the active project unless a prompt explicitly says otherwise.

## Active Project

- `AdafruitFiles/` is a PlatformIO Arduino project for an Adafruit Metro ESP32-S3.
- Main entry point: `AdafruitFiles/src/main.cpp`.
- PlatformIO config: `AdafruitFiles/platformio.ini`.
- Shared hardware constants live in `AdafruitFiles/include/hardware_config.h`.
- Older/reference code lives under `TeensyNewFiles/` and `TeensyOldFiles/`. Some old docs still describe a Teensy/Flask architecture, so verify paths before relying on them.

## Build Target and Dependencies

`AdafruitFiles/platformio.ini` defines:

- Platform: `espressif32`
- Board: `adafruit_metro_esp32s3`
- Framework: `arduino`
- Monitor speed: `115200`
- Libraries:
  - `ArduinoJson`
  - `Adafruit BNO08x`
  - `Adafruit BusIO`
  - `Adafruit Unified Sensor`
  - `Adafruit_VL53L0X`
  - `SparkFun LPS28DFW Arduino Library`

Typical build command from the repo root:

```powershell
cd AdafruitFiles
pio run
```

## Hardware Map

The active pin map is in `AdafruitFiles/include/hardware_config.h`.

- Motors / ESC outputs:
  - Front right: `D2`
  - Back right: `D3`
  - Back left: `D4`
  - Front left: `D5`
- ESC PWM:
  - `300 Hz`
  - `12-bit`
  - `1000 us` off/min
  - `2000 us` max
- BNO08x IMU over SPI:
  - CS: `D10`
  - INT: `D9`
  - RESET: `D8`
- Bluetooth serial:
  - `Serial1`
  - RX: `RX`
  - TX: `TX`
  - Baud: `9600`
- I2C sensors:
  - SDA: `SDA`
  - SCL: `SCL`
  - Clock: `400 kHz`
  - VL53L0X ToF address: `0x29`
  - LPS28DFW barometer uses its default I2C address
- HC-SR04 ultrasonic:
  - Trigger: `A0`
  - Echo: `A1`
  - Important: echo must be level shifted to 3.3 V before the ESP32-S3 pin.

## Runtime Architecture

`AdafruitFiles/src/main.cpp` performs boot setup:

1. Starts USB serial at `115200`.
2. Initializes Bluetooth, motors, IMU, and sensors.
   - If the IMU fails, `flightSystemReady` stays false and flight commands are rejected.
3. Creates three FreeRTOS tasks:
   - `flightTask` on `FLIGHT_CORE` / core `1`
   - `sensorTask` on `SENSOR_CORE` / core `0`
   - `telemetryTask` on `SENSOR_CORE` / core `0`
4. Leaves Arduino `loop()` idle with a 1 second delay.

The global `dt` is declared in `main.cpp` and exposed through `main.h`. `flightTask` updates it each loop.

## Flight Loop

Implemented in `AdafruitFiles/src/tasks.cpp`.

The flight task runs every `FLIGHT_TASK_PERIOD_US`, currently `2500 us` / roughly `400 Hz`.

Each loop:

1. Computes `dt`.
2. Calls `bluetoothPoll()`.
3. Calls `readIMU()`.
4. Calls `updateState()`.
5. Calls `calculatePID()`.
6. Calls `mixMotors()`.

If the loop finishes early, it delays for the remainder of the 2.5 ms period. If it overruns, it yields.

## Sensor Loop

Implemented in `AdafruitFiles/src/sensors.cpp`.

The sensor task runs every `SENSOR_TASK_PERIOD_MS`, currently `10 ms`.

Sensor polling behavior:

- Barometer updates every sensor-task loop.
- VL53L0X ToF updates every `50 ms`.
- ToF readings are treated as healthy only when the driver reports a valid range status and the distance is within plausible bounds.
- HC-SR04 ultrasonic trigger starts every `60 ms`.
- `publishSensorSnapshot()` copies latest readings into a shared `SensorSnapshot`.
- `areSensorsReady()` derives live sensor health from the protected shared snapshot.
- Access to the shared snapshot is protected with an ESP32 `portMUX_TYPE`.

`SensorSnapshot` is defined in `AdafruitFiles/include/sensors.h` and includes:

- `altitudeM`
- `pressureHpa`
- `ultrasonicCm`
- `tofCm`
- `barometerOk`
- `ultrasonicOk`
- `tofOk`
- `updatedMs`

## Bluetooth Protocol

Implemented in `AdafruitFiles/src/bluetooth.cpp`.

The firmware uses newline-delimited JSON over `Serial1`.

Incoming command shape:

```json
{"action":"forward","value":0}
```

Rules:

- `\n` terminates a message.
- `\r` is ignored.
- Receive buffer resets if it exceeds 256 characters.
- Invalid JSON is printed to USB serial and ignored.
- If no inbound control activity occurs for `BLUETOOTH_TIMEOUT_MS`, currently `5000 ms`, `bluetoothConnected` becomes `false`.
- Outbound telemetry does not refresh the Bluetooth timeout.
- `heartbeat` and `ping` messages refresh the control-link timeout without changing flight state.

Supported actions are handled in `AdafruitFiles/src/states.cpp`:

- `arm` (only accepted from `IDLE` when the IMU, control link, and at least one sensor are healthy)
- `disarm`
- `takeoff` (only accepted from `IDLE` when armed, IMU ready, the control link is active, and at least one sensor is healthy)
- `heartbeat`
- `ping`
- `cancel`
- `program_control`
- `land`
- `throttle`
- `up`
- `down`
- `forward`
- `backward`
- `left`
- `right`
- `stop_pitch`
- `stop_roll`

Important behavior: except for `arm`, `disarm`, and a valid `takeoff`, commands are ignored while the drone is in `IDLE`.

## State Machine

Implemented in `AdafruitFiles/src/states.cpp`.

States are declared in `AdafruitFiles/include/states.h`:

- `IDLE`
- `TAKEOFF`
- `USERCNTRL`
- `PRGMCNTRL`
- `LANDING`

Behavior:

- `IDLE` forces throttle, pitch, roll, and yaw targets to zero.
- `TAKEOFF` ramps throttle over `4000 ms` to `HOVER_THROTTLE_OFFSET_US`, currently `450 us`, then switches to `USERCNTRL`.
- `USERCNTRL` and `PRGMCNTRL` currently hold whatever command targets are set.
- `LANDING` zeros attitude targets and ramps throttle down from the starting throttle over `4000 ms`.
- Landing ends early and returns to `IDLE` if ultrasonic distance is valid and at or below `20 cm`.
- Landing now accepts either ultrasonic or ToF distance at or below `20 cm` as ground detection.
- If ground is not detected, landing ramps throttle down over `4000 ms`, briefly holds a low throttle, then stops the motors after the bounded landing timeout; `disarm` is still the explicit immediate stop path.
- If Bluetooth disconnects while not `IDLE` or `LANDING`, the firmware automatically enters `LANDING`.
- If the IMU becomes unavailable during runtime, flight is disabled and motors are stopped.
- If all sensors become unhealthy while armed in `IDLE`, the firmware clears the armed flag before takeoff can start.

## IMU and Attitude

Implemented in `AdafruitFiles/src/imu.cpp`.

- Uses an Adafruit BNO08x over SPI.
- Main attitude report: `SH2_GAME_ROTATION_VECTOR` at `2500 us`.
- Gyro report: `SH2_GYROSCOPE_CALIBRATED` at `2500 us`.
- Linear acceleration report is enabled at `5000 us`, but current code does not use it.
- If the BNO08x reports a reset during runtime, the firmware marks the IMU unavailable and disables flight until recalibration/reboot.
- If rotation-vector or gyro reports go stale for more than `100 ms`, `imuReady` becomes false and the flight task stops the motors.
- On initialization, the firmware averages up to 20 rotation-vector samples over up to 3 seconds to set roll, pitch, and yaw offsets.
- Quaternion-to-Euler conversion lives in `AdafruitFiles/src/utils.cpp`.

Global attitude values:

- `rollIMU`
- `pitchIMU`
- `yawIMU`
- `gyroX`
- `gyroY`
- `gyroZ`

## PID and Motor Mixing

PID is implemented in `AdafruitFiles/src/pid.cpp`.

Current defaults:

- Roll: `kp=1.5`, `ki=0.5`, `kd=0.05`
- Pitch: `kp=1.5`, `ki=0.5`, `kd=0.05`
- Yaw: `kp=2.0`, `ki=0.0`, `kd=0.05`
- Integral limit: `200`

PID output and integrators are held at zero until `throttleDesired >= PID_ACTIVE_THROTTLE_OFFSET_US`, currently `50 us`. Yaw error wraps across the `-180/180` degree boundary.

Motor output is implemented in `AdafruitFiles/src/motors.cpp`.

- `throttleDesired` is an offset above `PWM_OFF_US`.
- In `IDLE`, or below `PID_ACTIVE_THROTTLE_OFFSET_US`, all motors are written to `PWM_OFF_US`.
- Otherwise the base PWM is `PWM_OFF_US + throttleDesired`.
- Motor mix order is front-right, back-right, back-left, front-left:

```cpp
frontRight = base + rollPid - pitchPid + yawPid;
backRight  = base + rollPid + pitchPid - yawPid;
backLeft   = base - rollPid + pitchPid + yawPid;
frontLeft  = base - rollPid - pitchPid - yawPid;
```

## Telemetry

Implemented in `AdafruitFiles/src/telemetry.cpp`.

Sent every `500 ms` from a low-priority `telemetryTask` as newline-delimited JSON over Bluetooth.

Fields currently sent:

- `status`
- `pitch`
- `roll`
- `yaw`
- `target_pitch`
- `target_roll`
- `throttle`
- `armed`
- `battery` hardcoded to `100`
- `altitude_m`
- `pressure_hpa`
- `ultrasonic_cm`
- `tof_cm`
- `sensor_ok`
- `barometer_ok`
- `ultrasonic_ok`
- `tof_ok`
- `imu_ok`
- `flight_ready`
- `sensors_ready`
- `sensor_age_ms`
- `flight_age_ms`
- `bt_connected`

`sensor_ok` and `sensors_ready` are true if any one of barometer, ultrasonic, or ToF reports OK.
Barometer health is based on recent successful pressure reads, not only startup detection.

## PC Bridge

`AdafruitFiles/pcBluetoothComm.py` is a Flask plus PySerial bridge.

Behavior:

- Connects to `COM7` at `9600` by default; `SASE_DRONE_PORT` and `SASE_DRONE_BAUD` can override those values.
- Background thread reads newline-delimited serial data.
- Background heartbeat thread sends `{"action":"heartbeat","value":0}` once per second only while a bridge API client has touched `/api/status` or `/api/command` recently. `SASE_OPERATOR_TIMEOUT` controls the operator-presence window and defaults to `3.0` seconds.
- JSON telemetry updates `latest_drone_status`.
- Plain text serial lines are saved as `last_message`.
- Access to `latest_drone_status` is protected with a thread lock because Flask handlers and the serial listener share it.
- `GET /api/status` returns latest telemetry.
- `POST /api/command` sends compact command JSON to the drone.
- `GET /` returns bridge metadata as JSON.

There is currently no browser UI template under `AdafruitFiles/templates/`; the bridge is API-first.

## Common Edit Areas

- Hardware wiring or constants: `AdafruitFiles/include/hardware_config.h`
- Add/change commands: `AdafruitFiles/src/states.cpp`, possibly `AdafruitFiles/pcBluetoothComm.py`
- Add telemetry fields: `AdafruitFiles/src/telemetry.cpp`
- Change sensor polling or shared readings: `AdafruitFiles/src/sensors.cpp`
- Tune control loop: `AdafruitFiles/src/pid.cpp`
- Change motor layout/mixing: `AdafruitFiles/src/motors.cpp`
- Change boot/task scheduling: `AdafruitFiles/src/main.cpp` and `AdafruitFiles/src/tasks.cpp`

## Known Caveats

- The root `README.md` still describes an older BLE bridge/PWA stack under `TeensyOldFiles/`.
- `TeensyNewFiles/context.md` describes a Teensy 4.1 setup and should not be treated as current for `AdafruitFiles`.
- Battery telemetry is currently placeholder data.
- AQI sensor telemetry is not currently present in the active `AdafruitFiles` firmware despite the project name.
- Real ESC, motor direction, PID signs, and failsafe behavior need hardware validation before flight.
