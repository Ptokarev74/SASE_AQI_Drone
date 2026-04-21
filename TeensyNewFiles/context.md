# SASE AQI Drone Codebase Context

This document provides an overview of the SASE AQI Drone project architecture, components, and communication flow.

## Overview
The SASE AQI Drone project is a web-controlled quadcopter system. It uses a Python Flask server to bridge a web-based user interface with a C++ flight controller (running on a Teensy 4.1) via a serial Bluetooth connection. 

The user issues commands (like throttle adjustments, pitch/roll via WASD, or takeoff/land) from a browser. The Python server translates these into JSON payloads and sends them over Bluetooth. The drone parses the JSON, updates its state machine and target flight variables, computes PID loops, and outputs the necessary PWM signals to the motors. The drone also sends live telemetry data back to the web UI.

## Architecture

The project consists of three primary layers:

### 1. Frontend Web App (`templates/index.html`)
- **UI:** A sleek, dark-themed control dashboard with a D-pad for directional control (WASD keys mapped), takeoff/land buttons, and a throttle slider.
- **Communication:**
  - Sends control inputs via HTTP `POST` requests to `/api/command`. Key press events send commands (e.g., `forward`), and key release events send zeroing commands (e.g., `stop_pitch`) to self-level the drone.
  - Polls `/api/status` via a `GET` request every second to display real-time telemetry data (status, pitch, roll, etc.).

### 2. Backend Server (`pcBluetoothComm.py`)
- **Web Server:** A Flask application that serves the frontend UI.
- **API Endpoints:**
  - `/api/command`: Receives commands from the frontend, maps them to simpler actions/values, constructs a compact JSON payload, and writes it to the active Bluetooth COM port (default `COM7`).
  - `/api/status`: Returns the `latest_drone_status` dictionary.
- **Bluetooth Bridge:** Uses `pyserial` to connect to the paired Bluetooth module at 9600 baud. It runs a daemon thread (`listen_to_drone`) that constantly listens for incoming JSON telemetry data from the drone and updates the global state.

### 3. Flight Controller Firmware (`src/`, `include/`)
- Platform: Developed using Arduino framework for a Teensy 4.1 board, managed by PlatformIO (`platformio.ini`).
- Dependencies: `ArduinoJson` (JSON parsing/formatting), `Adafruit_BNO08x` (IMU), SparkFun LPS28DFW (Barometer).

#### Key Components:
- **`main.cpp`:** The core execution loop. It calculates `dt`, polls sensors (ultrasonic, barometer, IMU), checks for incoming Bluetooth messages, updates the state machine, calculates PID, and mixes motor outputs.
- **`states.cpp`:** The brain of the drone. It maintains a state machine (`IDLE`, `TAKEOFF`, `USERCNTRL`, `PRGMCNTRL`, `LANDING`). 
  - **Command Processing:** Parses incoming JSON commands via `ArduinoJson`. Maps string actions (e.g., `"forward"`, `"left"`, `"throttle"`) into specific pitch, roll, and throttle target values (`pitch_des`, `roll_des`, `thro_des`).
  - **Fail-Safe:** Automatically transitions to `LANDING` if the Bluetooth connection drops.
  - **Telemetry:** Periodically constructs a JSON document with current state, pitch, targets, etc., and sends it back over Bluetooth.
- **`bluetooth.cpp`:** Handles reading byte-by-byte from `Serial1` (where the HC-05/HC-06 module is connected), looking for newline (`\n`) terminators to form complete JSON strings. Also handles sending JSON responses.
- **Hardware Abstraction:** Modular C++ files manage specific hardware (`motors.cpp`, `imu.cpp`, `barometer.cpp`, `ultrasonic.cpp`) and math (`pid.cpp`).

## Communication Flow Diagram (JSON over Bluetooth)

1. **User Input:** User clicks "W" (Forward). JS sends `{ "action": "forward", "value": 2.0 }` to Flask.
2. **Bridge:** Flask minifies the JSON to `{"action":"forward","value":2.0}\n` and sends it via PySerial.
3. **Drone Receive:** `bluetooth.cpp` reads the serial stream until `\n`.
4. **Drone Process:** `states.cpp` deserializes the JSON. Since action is "forward", it sets `pitch_des = -20`.
5. **Control Loop:** `main.cpp` calls `calculatePID()` which compares the actual IMU pitch to the new `pitch_des` and adjusts motor speeds.
6. **Telemetry:** `states.cpp` packs current state and pitch into JSON and sends it back via `bluetooth.cpp`. Flask reads it, updates state, and the Web UI fetches it for display.
