import os
import serial
import json
import time
import threading
from typing import Any
from flask import Flask, request, jsonify

app = Flask(__name__)

# Update SASE_DRONE_PORT if your Bluetooth serial adapter appears elsewhere.
PORT = os.environ.get("SASE_DRONE_PORT", "COM7")
BAUD = int(os.environ.get("SASE_DRONE_BAUD", "9600"))
HEARTBEAT_INTERVAL_S = 1.0
OPERATOR_TIMEOUT_S = float(os.environ.get("SASE_OPERATOR_TIMEOUT", "3.0"))

# Global state
ser = None
serial_lock = threading.Lock()
operator_lock = threading.Lock()
status_lock = threading.Lock()
listener_started = False
heartbeat_started = False
last_operator_activity = 0.0
latest_drone_status: dict[str, Any] = {
    "status": "idle",
    "last_update": None
}

def mark_operator_activity() -> None:
    """Record that a client is actively using the bridge API."""
    global last_operator_activity
    with operator_lock:
        last_operator_activity = time.monotonic()

def operator_recently_active() -> bool:
    """Return true only while a frontend/API client is still present."""
    with operator_lock:
        return (
            last_operator_activity > 0.0 and
            time.monotonic() - last_operator_activity <= OPERATOR_TIMEOUT_S
        )

def write_json_to_drone(payload: dict[str, Any]) -> None:
    """Write one compact newline-delimited JSON message to the drone."""
    global ser
    if ser is None or not ser.is_open:
        raise serial.SerialException("Bluetooth not connected")

    json_payload = json.dumps(payload, separators=(',', ':'))
    with serial_lock:
        ser.write((json_payload + '\n').encode('utf-8'))

def listen_to_drone():
    """Background thread that constantly reads from the serial port"""
    global ser, latest_drone_status
    while True:
        if ser and ser.is_open:
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        try:
                            # Try to unpack JSON payload from the drone
                            data = json.loads(line)
                            with status_lock:
                                latest_drone_status.update(data)
                                latest_drone_status["last_update"] = time.time()
                            print(f"[Drone Telemetry]: {data}")
                        except json.JSONDecodeError:
                            # If the drone just sent plain text, save it
                            print(f"[Drone Serial]: {line}")
                            with status_lock:
                                latest_drone_status["last_message"] = line
            except Exception as e:
                print(f"[Listener Error]: {e}")
        
        # Short sleep to prevent CPU pegging
        time.sleep(0.05)

def heartbeat_to_drone():
    """Keep firmware control-link timeout fresh while an API client is active."""
    while True:
        if ser and ser.is_open and operator_recently_active():
            try:
                write_json_to_drone({"action": "heartbeat", "value": 0.0})
            except Exception as e:
                print(f"[Heartbeat Error]: {e}")
        time.sleep(HEARTBEAT_INTERVAL_S)

def init_serial():
    global ser, listener_started, heartbeat_started
    if ser is None or not ser.is_open:
        try:
            print(f"Connecting to Drone on {PORT}...")
            ser = serial.Serial(PORT, BAUD, timeout=2)
            time.sleep(2)  # Wait for connection to stabilize
            print("Connected!")
            
            # Start background threads as daemon so they close with the web server.
            if not listener_started:
                listener = threading.Thread(target=listen_to_drone, daemon=True)
                listener.start()
                listener_started = True

            if not heartbeat_started:
                heartbeat = threading.Thread(target=heartbeat_to_drone, daemon=True)
                heartbeat.start()
                heartbeat_started = True
            
        except serial.SerialException as e:
            print(f"Serial Error: {e}")
            ser = None

@app.route('/')
def index():
    return jsonify({
        "service": "SASE AQI Drone Bluetooth bridge",
        "serial_port": PORT,
        "baud": BAUD,
        "endpoints": ["/api/status", "/api/command"]
    })

@app.route('/api/status', methods=['GET'])
def get_status():
    """Endpoint for frontend to fetch the latest telemetry data"""
    mark_operator_activity()
    with status_lock:
        return jsonify(dict(latest_drone_status))

@app.route('/api/command', methods=['POST'])
def send_command():
    global ser
    mark_operator_activity()
    init_serial() # Ensure serial is connected
    
    if ser is None:
        return jsonify({"status": "error", "message": "Bluetooth not connected"}), 500

    data = request.get_json(silent=True) or {}
    action = data.get('action')
    value = data.get('value', 0.0)

    if not action:
        return jsonify({"status": "error", "message": "No action provided"}), 400

    # We send a simplified command payload to prevent Arduino 64-byte buffer overflow
    mission_data = {
        "action": action,
        "value": float(value)
    }

    try:
        print(f"Sending Mission: {mission_data}")
        write_json_to_drone(mission_data)
        
        # Revert 'last_message' so the UI displays success and then updates on next ping
        with status_lock:
            if "last_message" in latest_drone_status:
                latest_drone_status["last_message"] = "Waiting for response..."
            
        return jsonify({"status": "success", "message": f"Command {action} sent"})
    
    except Exception as e:
        print(f"Error sending command: {e}")
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == '__main__':
    # Initialize serial lazily or on startup
    init_serial()
    
    # disable reloader to prevent multiple threads from being launched
    app.run(debug=False, port=5000, use_reloader=False)
