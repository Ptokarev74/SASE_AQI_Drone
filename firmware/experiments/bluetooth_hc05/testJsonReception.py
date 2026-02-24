import serial
import json
import time

port = "COM8" # Update to your port
baud = 9600

try:
    ser = serial.Serial(port, baud, timeout=2)
    print("Connecting...")
    time.sleep(2)

    print("Sending Ping...")
    ser.write(b'P')
    
    # Read until the newline character we sent in Arduino
    raw_data = ser.readline().decode('utf-8').strip()
    
    if raw_data:
        # Parse the JSON string into a Python Dictionary
        data = json.loads(raw_data)
        
        print("\n--- Data Received ---")
        print(f"Status: {data['status']}")
        print(f"X: {data['position']['x']}")
        print(f"Y: {data['position']['y']}")
        print(f"Z: {data['position']['z']}")
    else:
        print("No data received.")

except Exception as e:
    print(f"Error: {e}")
finally:
    if 'ser' in locals():
        ser.close()
