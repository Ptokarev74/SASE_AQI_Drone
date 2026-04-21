import serial
import json
import time

# Update to your specific Teensy Bluetooth COM port
port = "COM3" 
baud = 9600

def send_drone_mission():
    try:
        # Connect to the Bluetooth module
        with serial.Serial(port, baud, timeout=2) as ser:
            print(f"Connected to Drone on {port}")
            time.sleep(2)  # Wait for Teensy/HC-05 to stabilize

            # Define your flight sequence
            mission_data = {
                "mission_id": 101,
                "commands": [
                    {"action": "up", "value": 2.0},
                    {"action": "right", "value": 2.0},
                    {"action": "right", "value": 5.0},
                    {"action": "land", "value": 0}
                ]
            }

            # Convert dictionary to a JSON string
            json_payload = json.dumps(mission_data)
            
            # Log the size for debugging
            print(f"Sending Mission ({len(json_payload)} bytes): {json_payload}")

            # Send the JSON followed by a newline so the Teensy knows it's the end
            ser.write((json_payload + '\n').encode('utf-8'))
            
            print("Mission sent successfully. Monitoring for confirmation...")
            
            # Optional: Listen for the Teensy to say "Mission Complete"
            while True:
                if ser.in_waiting > 0:
                    response = ser.readline().decode('utf-8').strip()
                    print(f"Drone: {response}")
                    if "Mission Complete" in response:
                        break
                time.sleep(0.1)

    except serial.SerialException as e:
        print(f"Serial Error: {e}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    send_drone_mission()