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
3. Creates two FreeRTOS tasks:
   - `flightTask` on `FLIGHT_CORE` / core `1`
   - `sensorTask` on `SENSOR_CORE` / core `0`
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
7. Sends telemetry every `TELEMETRY_PERIOD_MS`, currently `500 ms`.

If the loop finishes early, it delays for the remainder of the 2.5 ms period. If it overruns, it yields.

## Sensor Loop

Implemented in `AdafruitFiles/src/sensors.cpp`.

The sensor task runs every `SENSOR_TASK_PERIOD_MS`, currently `10 ms`.

Sensor polling behavior:

- Barometer updates every sensor-task loop.
- VL53L0X ToF updates every `50 ms`.
- HC-SR04 ultrasonic trigger starts every `60 ms`.
- `publishSensorSnapshot()` copies latest readings into a shared `SensorSnapshot`.
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
- If no Bluetooth activity occurs for `BLUETOOTH_TIMEOUT_MS`, currently `5000 ms`, `bluetoothConnected` becomes `false`.
- `bluetoothSendJson()` also marks Bluetooth as connected because successful writes count as activity in the current implementation.

Supported actions are handled in `AdafruitFiles/src/states.cpp`:

- `takeoff`
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

Important behavior: except for `takeoff`, commands are ignored while the drone is in `IDLE`.

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
- If Bluetooth disconnects while not `IDLE` or `LANDING`, the firmware automatically enters `LANDING`.

## IMU and Attitude

Implemented in `AdafruitFiles/src/imu.cpp`.

- Uses an Adafruit BNO08x over SPI.
- Main attitude report: `SH2_GAME_ROTATION_VECTOR` at `2500 us`.
- Gyro report: `SH2_GYROSCOPE_CALIBRATED` at `2500 us`.
- Linear acceleration report is enabled at `5000 us`, but current code does not use it.
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

The integrators only accumulate when `throttleDesired >= 50`.

Motor output is implemented in `AdafruitFiles/src/motors.cpp`.

- `throttleDesired` is an offset above `PWM_OFF_US`.
- In `IDLE`, all motors are written to `PWM_OFF_US`.
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

Sent every `500 ms` from `flightTask` as newline-delimited JSON over Bluetooth.

Fields currently sent:

- `status`
- `pitch`
- `roll`
- `yaw`
- `target_pitch`
- `target_roll`
- `throttle`
- `battery` hardcoded to `100`
- `altitude_m`
- `pressure_hpa`
- `ultrasonic_cm`
- `tof_cm`
- `sensor_ok`
- `bt_connected`

`sensor_ok` is true if any one of barometer, ultrasonic, or ToF reports OK.

## PC Bridge

`AdafruitFiles/pcBluetoothComm.py` is a Flask plus PySerial bridge.

Behavior:

- Connects to `COM7` at `9600`.
- Background thread reads newline-delimited serial data.
- JSON telemetry updates `latest_drone_status`.
- Plain text serial lines are saved as `last_message`.
- `GET /api/status` returns latest telemetry.
- `POST /api/command` sends compact command JSON to the drone.
- `GET /` renders `index.html`.

Note: there is no `AdafruitFiles/templates/index.html` visible in the current file list. `TeensyNewFiles/templates/index.html` exists and may be the older UI template this bridge expected.

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
- The Python bridge comment still says "Teensy Bluetooth COM port", but the active firmware target is the Metro ESP32-S3.
- Battery telemetry is currently placeholder data.
- AQI sensor telemetry is not currently present in the active `AdafruitFiles` firmware despite the project name.
- Real ESC, motor direction, PID signs, and failsafe behavior need hardware validation before flight.
