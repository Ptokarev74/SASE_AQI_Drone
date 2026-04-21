import serial
import json
import time
import threading
from typing import Any
from flask import Flask, request, jsonify, render_template

app = Flask(__name__)

# Update to your specific Teensy Bluetooth COM port
PORT = "COM7" 
BAUD = 9600

# Global state
ser = None
latest_drone_status: dict[str, Any] = {
    "status": "idle",
    "last_update": None
}

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
                            latest_drone_status.update(data)
                            latest_drone_status["last_update"] = time.time()
                            print(f"[Drone Telemetry]: {data}")
                        except json.JSONDecodeError:
                            # If the drone just sent plain text, save it
                            print(f"[Drone Serial]: {line}")
                            latest_drone_status["last_message"] = line
            except Exception as e:
                print(f"[Listener Error]: {e}")
        
        # Short sleep to prevent CPU pegging
        time.sleep(0.05)

def init_serial():
    global ser
    if ser is None or not ser.is_open:
        try:
            print(f"Connecting to Drone on {PORT}...")
            ser = serial.Serial(PORT, BAUD, timeout=2)
            time.sleep(2)  # Wait for connection to stabilize
            print("Connected!")
            
            # Start background listener thread as daemon so it closes when web server closes
            listener = threading.Thread(target=listen_to_drone, daemon=True)
            listener.start()
            
        except serial.SerialException as e:
            print(f"Serial Error: {e}")
            ser = None

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/status', methods=['GET'])
def get_status():
    """Endpoint for frontend to fetch the latest telemetry data"""
    return jsonify(latest_drone_status)

@app.route('/api/command', methods=['POST'])
def send_command():
    global ser
    init_serial() # Ensure serial is connected
    
    if ser is None:
        return jsonify({"status": "error", "message": "Bluetooth not connected"}), 500

    data = request.json
    action = data.get('action')
    value = data.get('value', 0.0)

    if not action:
        return jsonify({"status": "error", "message": "No action provided"}), 400

    # Map the action to a specific mission ID
    action_to_id = {
        'forward': 1,
        'right': 2,
        'backward': 3,
        'left': 4,
        'up': 5,
        'down': 6,
        'takeoff': 7,
        'land': 8,
        'throttle': 9
    }
    
    m_id = action_to_id.get(action, 99) # Default to 99 if action is not mapped

    # We send a simplified command payload to prevent Arduino 64-byte buffer overflow
    mission_data = {
        "action": action,
        "value": float(value)
    }

    try:
        # Strip spaces to compress payload size further
        json_payload = json.dumps(mission_data, separators=(',', ':'))
        print(f"Sending Mission: {json_payload}")
        
        # Send payload + newline over bluetooth
        ser.write((json_payload + '\n').encode('utf-8'))
        
        # Revert 'last_message' so the UI displays success and then updates on next ping
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
    app.run(debug=True, port=5000, use_reloader=False)