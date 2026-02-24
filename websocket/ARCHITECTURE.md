# AQI Drone Bridge — Architecture & Hardening Guide

## Module Map

```
bridge/
├── __init__.py          # package marker
├── __main__.py          # python -m bridge entrypoint
├── config.py            # ALL tuneable constants (UUIDs, rates, queue sizes)
├── schemas.py           # Pydantic v2 models for telemetry & control
├── ble_client.py        # Async BLE central (bleak) — scan, connect, notify, write
├── server.py            # FastAPI WebSocket server — /ws, /health, broadcast loop
├── main.py              # Orchestrator — 5 asyncio tasks, deadman, shutdown
└── requirements.txt     # pip dependencies
```

## Concurrency Model

Five `asyncio.Task`s run on a **single event loop**:

| Task | Role | Failure recovery |
|------|------|-----------------|
| `ble_loop` | Scan → connect → wait-for-disconnect → retry | Infinite retry with backoff |
| `uvicorn` | HTTP + WebSocket server | Restarts on fatal error |
| `cmd_consumer` | Drain command queue → BLE write | Drops write on disconnect, resumes on reconnect |
| `telem_broadcast` | Fan-out telemetry to all WS clients | Prunes dead sockets, logs errors, continues |
| `deadman` | Sends ZERO_COMMAND if pilot goes silent | Always running, fires once per timeout window |

## Hardened Telemetry Protocol (Arduino → Python)

To eliminate fragmentation bugs, the bridge enforces a strict **Newline-Delimited JSON (NDJSON)** contract:

1. **Framing**: Every JSON message MUST be terminated with a single `\n` (newline) character.
2. **No Embedded Newlines**: The JSON payload itself must not contain newline characters.
3. **Max Message Size**: 1024 bytes (discarded if exceeded).
4. **Max Buffer Size**: 4096 bytes (reset if exceeded without a newline).

### Reassembly Mechanism
The Python BLE client maintains a **persistent byte buffer** per connection. Incoming data is appended raw. A processing loop splits the buffer on `b'\n'`, extracts complete frames, and leaves partial data in the buffer for the next notification. This handles:
- **MTU Fragmentation**: Large JSONs split across multiple packets.
- **Packet Bunching**: Multiple JSONs arriving in a single BLE event.
- **Partial Frames**: The tail end of a message waiting for the remainder.

## Failure Modes

| Failure | Detection | Response |
|---------|-----------|----------|
| BLE device not found | Scan returns 0 matches | Log + retry scan after `BLE_RECONNECT_DELAY_S` |
| BLE disconnect mid-flight | `disconnected_callback` fires | Clear `connected` event, flush buffer, auto-reconnect |
| UTF-8 Decode Error | `UnicodeDecodeError` | Log error, discard frame, continue |
| Telemetry JSON corrupt | `json.JSONDecodeError` | Log debug, skip frame, wait for next newline |
| Telemetry message too large| Frame > 1024 bytes | Log warning, discard frame |
| Telemetry buffer overflow | Buffer > 4096 bytes | Log warning, reset buffer to prevent memory leak |
| Command queue full | `queue.full()` | Drop **oldest** command (pop front), enqueue newest |
| WS client crash | `send_text()` raises | Remove from client set, continue broadcast |
| Pilot stops sending | `deadman_timer` detects elapsed > timeout | Write `ZERO_COMMAND` to BLE |
| Bridge process killed | SIGINT / SIGTERM | Cancel all tasks, disconnect BLE, exit cleanly |

## Command Flooding Prevention

```
PWA (30 Hz) → WS → validate → bounded Queue(4) → consumer → BLE write
```

- The queue is bounded to `COMMAND_QUEUE_SIZE` (default 4).
- When the queue is full, the **oldest** entry is dropped so the drone always receives the most recent intent.
- Only **one** consumer task serialises BLE writes, preventing concurrent GATT operations.
- BLE write-without-response is used to minimise round-trip latency.

## WebSocket Security

Simple bearer-token authentication:

```
ws://host:8765/ws?token=YOUR_SECRET
```

- Set `WS_AUTH_TOKEN` in `config.py` to enable.
- If the token doesn't match, the server closes the socket with code `4003 Forbidden`.
- Set it to `None` to disable auth (useful during development).
- For production, layer TLS (wss://) via a reverse proxy (nginx / Caddy).

## Testing Strategy

### Unit Tests (no hardware)
- **Schema validation**: Construct `TelemetryMessage` and `ControlCommand` with valid/invalid data; assert Pydantic raises on bad input.
- **Buffer reassembly**: Feed chunks to `_on_telemetry_notification`, verify `latest_telemetry` is set correctly.
- **Deadman timer**: Mock `time.monotonic()`, assert `ZERO_COMMAND` is written after timeout.

### Integration Tests (no hardware)
- **Mock BLE**: Use `unittest.mock.AsyncMock` to replace `BleakClient`. Simulate connect/disconnect cycles, trigger notifications.
- **WS round-trip**: Use `httpx.AsyncClient` + `websockets` to connect to the FastAPI app, send commands, verify echoed telemetry.

### Hardware Simulation
- Use an **nRF52840 DK** or **ESP32** running a minimal BLE peripheral sketch that advertises the same UUIDs and sends canned telemetry at 30 Hz.
- Alternatively, use [bless](https://github.com/kevincar/bless) to create a virtual BLE peripheral on a second machine.

## Performance Tuning

| Knob | Location | Default | Effect |
|------|----------|---------|--------|
| `TELEMETRY_BROADCAST_HZ` | `config.py` | 30 | Lower = less CPU, higher = smoother UI |
| `COMMAND_QUEUE_SIZE` | `config.py` | 4 | Lower = fresher commands, higher = more burst tolerance |
| `DEADMAN_TIMEOUT_S` | `config.py` | 0.5 s | Lower = faster emergency stop, higher = tolerates jitter |
| `BLE_RECONNECT_DELAY_S` | `config.py` | 3 s | Lower = faster reconnect, higher = less scan spam |
| `BLE_SCAN_TIMEOUT_S` | `config.py` | 10 s | Higher = more reliable discovery in noisy RF environments |

## Running the Bridge

```powershell
cd c:\Users\lev52\Desktop\SASE_AQI_Drone
pip install -r bridge/requirements.txt
python -m bridge.main
```

The server starts on `http://0.0.0.0:8765`. Health check: `GET /health`. WebSocket: `ws://localhost:8765/ws`.
