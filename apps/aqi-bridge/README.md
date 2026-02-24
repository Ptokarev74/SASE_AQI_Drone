# SASE AQI Drone: Bridge Service

This repository contains the `aqi_bridge` component, a Python service that securely links the drone's low-latency BLE (Bluetooth Low Energy) telemetry feed to the high-throughput WebSocket API consumed by the frontend Progressive Web App (PWA).

## Features
- Translates fragmented BLE byte-streams into unified WebSocket JSON streams.
- Manages command serialization and concurrent queue policies.
- Protects clients using strict WebSocket authentication modes (`disabled`, `telemetry_only`, `required`) and constant-time token comparisons.
- Designed with strict 'Fail-Fast' Supervisor exit policies.
- Comprehensive `pytest` coverage validating architecture stability.

## How to Run

### 1. Prerequisites
- Python 3.13+ installed (`$env:PYTHONPATH = .` must be set)
- Valid Bluetooth interface adapter.

### 2. Development Mode
To launch the bridge locally on a trusted LAN for development (authentication disabled):
```powershell
# Set Python path to the project root directory
$env:PYTHONPATH = "."

# Start the bridge explicitly using the canonical module entry point
cd apps/aqi-bridge
python -m aqi_bridge
```

### 3. Production Mode
In production, the bridge MUST be configured with a secure token and run behind a reverse proxy (e.g., Nginx) for TLS termination. 

```bash
# Configure strict auth policies before executing
export WS_AUTH_MODE="required"
export WS_AUTH_TOKEN="your-secure-production-token"

python3 -m aqi_bridge
```
*Note: Refer to `architecture.md` (Runbook section) for a complete systemd and Nginx deployment guide.*

### Expected Output
On successful startup with mocked or real BLE connections, your terminal will display explicit initialization confirmation:
```
INFO:aqi_bridge.api:WebSocket auth mode: disabled, token_rotation_allowed: False
INFO:uvicorn.error:Started server process [1234]
INFO:uvicorn.error:Waiting for application startup.
INFO:uvicorn.error:Application startup complete.
INFO:uvicorn.error:Uvicorn running on http://127.0.0.1:8765 (Press CTRL+C to quit)
```

## Quick Start Smoke Test
Once the bridge reports "Uvicorn running", open a new terminal and verify health telemetry via REST.

```powershell
# Query the live telemetry and API health endpoint
Invoke-WebRequest -Uri http://localhost:8765/health | Select-Object -ExpandProperty Content
```

**Expected JSON Response:**
```json
{
  "ble_connected": false,
  "negotiated_mtu": 23,
  "usable_payload_bytes": 20,
  "chunking_enabled": false,
  "auth": {
    "mode": "disabled",
    "token_configured": false
  },
  ...
}
```

*For complete implementation tradeoffs, deployment constraints, and the internal framing protocol, please review the [architecture.md](architecture.md) and [protocol.md](protocol.md) documents.*
