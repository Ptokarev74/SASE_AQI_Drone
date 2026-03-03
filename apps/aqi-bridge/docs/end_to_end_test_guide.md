# SASE AQI Drone: End-to-End Bridge Testing Guide

This guide explains how to test the full communication pipeline of the `aqi-bridge` without needing the actual drone hardware or the final React PWA wrapper. It uses a mock Arduino sketch to simulate the drone over Bluetooth and a raw HTML client to simulate the pilot's browser over WebSockets.

## 1. Prepare the Arduino (The Mock Drone)

First, we need to load a script onto an Arduino that pretends to be the drone. It will broadcast the exact BLE name (`AQI_Drone`) and UUIDs the Python bridge expects.

1. Open the Arduino IDE.
2. Open the file `firmware/bridge_test/bridge_test.ino`
3. Ensure you have the `ArduinoBLE` library installed.
4. Flash the code to a BLE-capable Arduino board (e.g., Nano 33 BLE, ESP32).
5. Open the **Arduino Serial Monitor** (Baud: 115200). You should see:
   ```text
   --- AQI Drone Bridge Test ---
   BLE Advertising as 'AQI_Drone'...
   ```

## 2. Configure and Start the Python Bridge

The Python bridge acts as the middleman, translating WebSockets to Bluetooth.

### Important Note on CRC Checksums!
By default, the bridge enforces **Protocol Hardening** (`BLE_CRC_REQUIRED = True` in `config.py`). This means it will instantly drop any telemetry from the Arduino that does not end in a `|` followed by a valid hex CRC32 checksum. 

Because our simple `bridge_test.ino` mockup just sends raw JSON (`{"aqi":50...}\n`) without calculating a CRC, **you must temporarily disable CRC checking to run this test.**

1. Open `apps/aqi-bridge/aqi_bridge/config.py`.
2. Find the line `BLE_CRC_REQUIRED: bool = True` (around line 117).
3. Change it to: `BLE_CRC_REQUIRED: bool = False`.
4. *Remember to change this back to `True` before actual deployment!*

Now, start the bridge:
```powershell
# From the apps/aqi-bridge directory
py -m aqi_bridge
```

You should see the bridge find the Arduino, negotiate the MTU, and subscribe to telemetry:
```text
INFO      aqi_bridge.ble  BLE scanning for drone...
INFO      aqi_bridge.ble  Found device: AQI_Drone 
INFO      aqi_bridge.ble  Connecting to ...
INFO      aqi_bridge.ble  MTU negotiated: 247 bytes
```

## 3. Connect the HTML Client (The Mock Pilot)

Now we connect a client to the bridge to view the data and send commands.

1. Ensure the Python bridge from Step 2 is still running in the background.
2. In your File Explorer, navigate to: `apps/aqi-bridge/tests/html_client/`
3. **Double-click `index.html`** to open it in your web browser (Chrome, Edge, etc.).

### Verification A: Telemetry (Drone -> Bridge -> Browser)
Look at the HTML page. 
* The status badge should say **"Connected"** (Green).
* The "Latest Telemetry" box should be rapidly scrolling with JSON data.
* You should see the `aqi` and `batt_v` values changing slightly every second. This proves data is successfully flowing from the Arduino loop into your browser.

### Verification B: Control Commands (Browser -> Bridge -> Drone)
1. Click the blue **"Send Arm Command"** button on the HTML page. 
2. *Behind the scenes: The HTML page starts sending a JSON command `{vx: 0, yaw: 0, arm: true}` 20 times a second to keep the bridge's deadman switch happy.*
3. Look at your **Arduino Serial Monitor**. You should see a flood of commands arriving:
   ```text
   --- Command Received ---
   vx:  0.00
   vy:  0.00
   vz:  0.00
   yaw: 0.00
   arm: TRUE
   crc: 0
   ```
4. Click the red **"Send Disarm Command"** button on the HTML page.
5. The Arduino Serial Monitor should instantly switch to printing `arm: FALSE`.

## 4. Alternative: Sending Scripted Commands directly (No Browser)

If you want to test sending commands straight from Python over Bluetooth (bypassing the WebSocket server entirely), you can use the standalone script we created.

1. **Stop the main bridge** by pressing `CTRL+C` in its terminal. (You cannot have the main bridge and the test script fighting over the Bluetooth adapter).
2. Ensure the Arduino is still powered on and advertising.
3. Run the direct test script:
   ```powershell
   py test_bridge_to_arduino.py
   ```
4. Watch the Arduino Serial Monitor. The Python script will connect and slowly step through a pre-programmed sequence of commands (Arm, Pitch, Strafe, Ascend, Yaw, Disarm), waiting 2 seconds between each. You will see the floating point values change on the Arduino exactly as scripted.

## 5. Cleanup
When finished, stop the Python bridge (`CTRL+C`) and **remember to revert `BLE_CRC_REQUIRED = True`** in `config.py` to re-enable production security rules.
