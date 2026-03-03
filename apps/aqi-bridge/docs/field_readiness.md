# Field Readiness & Testing Guide

This guide outlines how to verify that the SASE AQI Drone bridge is ready for field deployment.

## 1. Automated Verification (Software Integrity)

Before going to the field, ensure the software stack is internally consistent.

### Pytest Suite
Run the full suite of 75 tests. These cover WebSocket authentication, BLE reassembly, command queue policies (drop-oldest), and broad system limits.

```powershell
# From the project root
$env:PYTHONPATH="apps/aqi-bridge"
py -m pytest apps/aqi-bridge/tests/ -v
```

### Static Analysis
Ensure there are no linting or type errors that could cause runtime crashes.

```powershell
# From the project root
py -m ruff check apps/aqi-bridge/aqi_bridge/
```

## 2. Mock Hardware Integration (Protocol Validation)

If the real hardware is unavailable, validate the bridge against the protocol.

1. **Launch the Bridge**:
   ```powershell
   cd apps/aqi-bridge
   python -m aqi_bridge
   ```
2. **Verify API Health**:
   Open `http://localhost:8765/health` in your browser. Verify `ble_connected` is false and `auth.mode` matches your config.

## 3. Field Testing Checklist (Hardware Integration)

When at the flight field, follow these steps in order:

### Phase A: Radio & Connection
- [ ] **Discovery**: Ensure the bridge consistently discovers the drone (check `BLE_DEVICE_NAME` in `config.py`).
- [ ] **MTU Negotiation**: Connect and check `/health`. Verify `negotiated_mtu` is > 200. If it stays at 23, ensure the drone firmware is functioning.
- [ ] **Stability**: Move the bridge computer 10-20 meters away. Ensure `ble_connected` remains true and telemetry continues to flow at ~30Hz.

### Phase B: Safety Mechanisms
- [ ] **Deadman Trigger**: While the drone is armed (on the ground!), disconnect the pilot's phone. Verify the bridge logs an emergency ZERO_COMMAND within 500ms.
- [ ] **Queue Saturation**: Use the PWA to send rapid commands. Verify the bridge doesn't crash or lag; it should drop older commands to preserve fresh intent.

### Phase C: Data Integrity
- [ ] **NDJSON Framing**: Ensure AQI and GPS data in the PWA dashboard doesn't "flicker" or show partial strings. This confirms the reassembly buffer is working.
- [ ] **Battery/Signal Range**: Monitor the signal RSSI. If signal drops, ensure the bridge auto-reconnects as defined in `architecture.md`.

## 4. Deployment Verification

If running in Docker (Production):
```bash
docker build -t sase-aqi-bridge:ci apps/aqi-bridge/
docker run -it --network host sase-aqi-bridge:ci
```
*Note: `--network host` is required on Linux for BLE access.*
