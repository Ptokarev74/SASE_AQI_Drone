# SASE AQI Drone

Welcome to the SASE AQI Drone project! This repository contains the complete software stack for a drone equipped with Air Quality Index (AQI) sensors, controlled via a web interface.

## System Overview

The project is composed of three main components working together in real time:

1. **Arduino (Drone Firmware)**: Collects sensor data (telemetry) and controls the drone's flight based on received commands. Lives on the physical drone.
2. **Python Bridge**: A local service running on a computer that acts as the strict middleman between the drone's Bluetooth radio and the web.
3. **Progressive Web App (PWA)**: The user interface running on a phone or laptop, providing joystick controls and a dashboard for telemetry data.

---

## How It Works: The Python Bridge

A “Python bridge” (`apps/aqi-bridge/`) is a local service on your laptop that sits between two links:

*   **BLE link**: laptop ↔ Arduino (telemetry + commands)
*   **Web link**: laptop ↔ phone PWA (telemetry + joystick commands)

It does exactly four jobs, and nothing else:

1. Connect and automatically reconnect to the Arduino over Bluetooth Low Energy (BLE).
2. Subscribe to telemetry notifications and parse the incoming JSON data.
3. Accept control commands from the PWA via a high-throughput WebSocket.
4. Forward those commands to the Arduino over BLE (via characteristic writes).

## Architecture

### 1) Process Diagram

```text
      Arduino (BLE Peripheral)                             Python Bridge (BLE Central)                              PWA UI (Phone/Browser)
                 |                                                      |                                                     |
  [Sensors] ---> | ----- Telemetry (Notify Char) ---------------------> | ----- Telemetry (WebSocket) ----------------------> | [Dashboard] 
  [Motors]  <--- | <---- Command (Write Char) ------------------------- | <---- Joystick (WebSocket) ------------------------ | [Touch Controls]
                 |                                                      |                                                     |
```

### 2) BLE Side Contract

The system uses a custom GATT service with two primary characteristics representing the contract between the Arduino and the Laptop:

*   **`TELEMETRY_CHAR` (Notify)**
    *   **Payload**: JSON bytes (or newline-delimited JSON if the device chunks large payloads).
    *   **Direction**: Arduino ➔ Bridge.

*   **`COMMAND_CHAR` (Write)**
    *   *Prefer “Write Without Response” for absolute minimum latency.*
    *   **Payload**: Compact JSON command or binary control structure.
    *   **Direction**: Bridge ➔ Arduino.

## Directory Structure
*   `apps/aqi-bridge/`: The Python BLE-to-WebSocket bridge service.
*   `apps/pwa/`: The frontend Progressive Web App interface.
*   `firmware/`: Arduino sketches and hardware experiments.
*   `docs/`: Detailed technical documentation (`architecture.md`, `protocol.md`, `field_readiness.md`).
