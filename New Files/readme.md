# SASE Drone Bluetooth Communication

The goal of this project is to establish Bluetooth communication with a drone to send and receive movement commands.

## Current State

The codebase consists of three main architectural components:
1. **Web UI (`templates/index.html`)**: An interactive frontend dashboard with control buttons that trigger POST requests to the Python server.
2. **Python Web/Control Server (`pcBluetoothComm.py`)**: A Flask web app that serves the frontend, handles POST requests at `/api/command`, maintains an active paired Bluetooth COM port, and bridges the web commands into JSON flights.
3. **Arduino Drone Receiver (`sketch_apr6a/sketch_apr6a.ino`)**: Runs on the drone itself, listening for the JSON sequence over a Bluetooth connection (`Serial1`). It parses the incoming missions and extracts specific flight actions and durations.

### Features Implemented

#### Web UI & Python Server (`pcBluetoothComm.py`)
- **Web Dashboard:** Aesthetic control pad for sending manual drone commands with clear UI feedback.
- **Flask API:** Exposes a `/api/command` POST route to trigger remote movement.
- **Bluetooth Connection:** Establishes communication via the `pyserial` library (defaulting to `COM7` at 9600 baud).
- **Payload Transmission:** Converts action inputs into the established JSON schema and sends it via serial format line terminated (`\n`).

### Startup Instructions
1. First, ensure you have the required Python dependencies: `pip install flask pyserial`
2. Change the `PORT` variable in `pcBluetoothComm.py` to match the serial port of your PC's active Bluetooth connection to the drone.
3. Start the local server: `python pcBluetoothComm.py`
4. Navigate your browser to `http://127.0.0.1:5000` to view the Drone Control Panel.

#### Arduino Receiver (`sketch_apr6a.ino`)
- **Serial Communication:** 
  - Initializes the main serial interface (`Serial`) at 115200 baud for debugging and logging to the connected computer.
  - Initializes a Bluetooth serial interface (`Serial1` using pins 0 & 1) at 9600 baud to receive data.
- **JSON Parsing:** 
  - Listens for incoming data on the Bluetooth connection.
  - Uses the `ArduinoJson` library to deserialize incoming JSON payloads.
- **Command Processing:**
  - Extracts a `mission_id` integer to identify the incoming flight plan.
  - Extracts an array of `commands`.
  - Loops through the command array, pulling out each `action` (string) and its corresponding `value` (float representing the duration in seconds).
  - Logs the received actions to the serial monitor.
  - Currently simulates execution using a simple `delay` block for testing purposes.

### Next Steps
- Implement actual drone movement functions (e.g., `moveDrone(action, duration)`).
- Replace the dummy `delay` logic with real control signals sent to the drone's flight controller or motors.

### Dependencies
- **[ArduinoJson](https://arduinojson.org/)**: Required to process the incoming payload. Ensure this library is installed via the Arduino IDE Library Manager.

### Hardware Prerequisites
- Arduino-compatible board with at least two Hardware Serial ports (e.g., Arduino Mega, Leonardo, etc.) or utilizing SoftwareSerial if pins 0 & 1 are occupied by the main computer connection.
- Bluetooth Module (e.g., HC-05, HC-06) connected to the board's `Serial1` pins configured to 9600 baud.

### Example JSON Payload Structure
To test the current implementation, standard payloads should look like this:

```json
{
  "mission_id": 1,
  "commands": [
    {
      "action": "takeoff",
      "value": 5.0
    },
    {
      "action": "forward",
      "value": 2.5
    },
    {
      "action": "land",
      "value": 0.0
    }
  ]
}
```
