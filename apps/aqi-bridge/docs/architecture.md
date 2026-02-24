# AQI Drone Bridge — Architecture & Hardening Guide

*This system prioritizes bounded latency, safety, and deterministic failure containment over throughput, flexibility, and absolute completeness.*

## Module Map

```
apps/aqi-bridge/
├── aqi_bridge/
│   ├── __init__.py          # package marker
│   ├── __main__.py          # python -m aqi_bridge entrypoint
│   ├── config.py            # ALL tuneable constants (UUIDs, rates, queue sizes)
│   ├── models.py            # Pydantic v2 models for telemetry & control
│   ├── ble.py               # Async BLE central (bleak) — scan, connect, notify, write
│   ├── api.py               # FastAPI WebSocket server — /ws, /health, broadcast loop
│   └── app.py               # Orchestrator — 5 asyncio tasks, deadman, shutdown
├── tests/                   # Pytest suite
├── requirements.txt         # pip dependencies
├── Dockerfile
└── docker-compose.yml
```

## Concurrency Model

Five `asyncio.Task`s run on a **single event loop**:

| Task | Role | Failure recovery |
|------|------|-----------------|
| `ble_loop` | Scan → connect → wait-for-disconnect → retry | Infinite retry with backoff |
| `uvicorn` | HTTP + WebSocket server | **Fail-Fast**: exits process on crash (supervised) |
| `cmd_consumer` | Drain command queue → BLE write | Resumes on BLE reconnect |
| `telem_broadcast` | Fan-out telemetry to all WS clients | Prunes dead sockets, continues |
| `deadman` | Sends ZERO_COMMAND if pilot goes silent | Always running |

## Design Tradeoffs

Engineering decisions in this bridge navigate constraints inherent to real-time control over lossy RF links. The following explicit tradeoffs dictate the architecture:

1. **Queue Size `4` + Drop-Oldest Policy**
   - *Constraint*: The pilot's phone produces WebSocket commands faster than the BLE GATT stack can transmit them.
   - *Failure Mode*: A deep queue creates unbounded latency. The drone executes a command from seconds ago, ignoring current stick inputs, leading to a crash.
   - *Tradeoff*: **Bounded Latency > Command Completeness**. 
   - *Result*: We intentionally drop stale commands. At 30Hz, a queue length of 4 caps worst-case command age at ~130ms.

2. **Single-Writer Task (No Parallel Writes)**
   - *Constraint*: OS bluetooth stacks and the Arduino peripheral cannot handle concurrent GATT write requests without race conditions or driver crashes.
   - *Failure Mode*: Two coroutines write simultaneously, interleaving bytes and causing unpredictable hardware states.
   - *Tradeoff*: **Safety > Parallel Throughput**.
   - *Result*: All writes are forced through a single bottleneck queue (`command_consumer_loop`). Emergency stop (`deadman`) commands wait in line behind maximum 4 normal commands, taking <130ms, rather than risking a hardware lockup.

3. **Newline-Delimited JSON (NDJSON) Framing**
   - *Constraint*: BLE MTUs are generally smaller (e.g., 20-250 bytes) than the telemetry payload (200-400 bytes).
   - *Failure Mode*: The Python client receives a stream of partial chunks and cannot deterministically identify where one payload ends and the next begins.
   - *Tradeoff*: **Structural Simplicity > Binary Compactness**.
   - *Result*: Instead of complex COBS encoding or length-prefixing, the Arduino terminates every payload with `\n`. The Python client reads until `\n`. It uses slightly more bandwidth but is radically easier to debug and virtually immune to framing desynchronization.

4. **Deadman Timeout at `0.5s`**
   - *Constraint*: Network jitter over mobile connections may delay ping packets unpredictably.
   - *Failure Mode*: The drone continues flying forward blindly while the phone is disconnected.
   - *Tradeoff*: **Safety Stop > Jitter Tolerance**.
   - *Result*: Set aggressively fast (500ms). If the pilot's commands are delayed by a half-second, the bridge forcefully arrests the drone. This may cause occasional stuttering on bad networks, but absolutely prevents flyaways.

5. **Immutable Telemetry Replacement**
   - *Constraint*: `broadcast_telemetry_loop` reads data 30 times a second while `ble_loop` updates data at unpredictable intervals.
   - *Failure Mode*: Serializing a dictionary while it is being updated causes a `RuntimeError: dictionary changed size during iteration` or results in mixed/stale telemetry fields.
   - *Tradeoff*: **Memory Allocation > Lock Contention**.
   - *Result*: We do **not** hold a lock during the slow JSON serialization. Instead, `ble_loop` allocates a brand-new object for every update, allowing the broadcast loop to grab an immutable reference under lock in < 1µs.

## BLE MTU Strategy

Bluetooth Low Energy MTU sizes dictate how much physical data can be sent in a single packet. A mismatch between expected and actual MTU results in truncated data and silently dropped packets.

- **Default ATT MTU:** 23 bytes (3 bytes overhead = **20 bytes usable payload**).
- **Target Negotiated MTU:** 247 bytes.

The bridge performs runtime MTU awareness and enforcement:

| Link Condition | Mode | Behavior |
|----------------|------|----------|
| **`Usable Payload >= 185`** | **Direct (NDJSON)** | The Arduino sends full JSON strings terminated by `\n`. |
| **`Usable Payload < 185`** | **Chunked** | The system seamlessly degrades. The Arduino explicitly splits JSON into indexed frames (`<seq>|<total>|<idx>|data\n`). |

**Hard Enforcement (`MAX_TELEMETRY_JSON_BYTES`):**
Regardless of chunking, if the fully reassembled JSON telemetry string exceeds the administratively defined `MAX_TELEMETRY_JSON_BYTES` (1024 bytes), the bridge completely drops and rejects the payload to prevent memory exhaustion and buffer overflow attacks. See `PROTOCOL.md` for exact message formats.

## Locking Rules

All shared-state access follows a strict contract to prevent BLE stalls:

| Rule | What it means |
|------|---------------|
| **Lock = memory only** | `ble.telemetry_lock` is held for a single Python reference copy (`telem = ble.latest_telemetry`). Estimated hold time < 1 µs. |
| **No I/O inside a lock** | JSON serialisation, `ws.send_text()`, and client pruning all happen *after* the lock is released. A stalled client cannot block BLE processing. |
| **Per-client send timeout** | Every `ws.send_text()` is wrapped in `asyncio.wait_for(timeout=WS_SEND_TIMEOUT_S)`. If a client exceeds the deadline it is removed immediately. |
| **Registry is asyncio-safe** | The client `set` is only ever mutated from the single-threaded asyncio event loop. A `list(clients)` snapshot is taken before iteration so the set can be safely modified during the loop. |

## BLE Write Serialization Rule

> **All BLE writes are serialized through a single command consumer coroutine; no other code path may call BLE write APIs.**

```
ALLOWED:   command_consumer_loop  →  ble._write_command_bytes()
FORBIDDEN: deadman_timer          →  ble._write_command_bytes()  ← BUG
FORBIDDEN: websocket_endpoint     →  ble._write_command_bytes()  ← BUG
FORBIDDEN: any other coroutine    →  ble._write_command_bytes()  ← BUG
```

The deadman timer and any other emergency handler must **enqueue** via `enqueue_command_drop_oldest()`. The consumer loop is the gate.

### Defense-in-Depth Enforcement Layers

| Layer | Mechanism | Purpose |
|-------|-----------|---------|
| **Naming** | Method named `_write_command_bytes` (underscore prefix) | Signals internal-only API to future maintainers |
| **Lock** | `asyncio.Lock` (`_write_lock`) wraps every `write_gatt_char` call | Serializes any accidental concurrent caller |
| **Counter** | `writes_in_flight` incremented before / decremented after every write | Logs `ERROR` if `> 1` — detects concurrency bug immediately |
| **Docstring ⚠** | `WARNING: NOT SAFE FOR CONCURRENT USE` comment above method | Explains the rule at the point of temptation |
| **Architecture doc** | This section | Developer reference |

## WebSocket Close-Code Policy

To ensure deterministic behavior and protocol consistency, this bridge explicitly standardizes on a **single** WebSocket close code for all authentication failures: **`1008` (Policy Violation)**.

- Per **RFC 6455 §7.4.1**, `1008` is the correct protocol-level signal indicating a generic policy violation (which includes authentication/authorization rejections).
- The use of unstandardized 4000-range codes (e.g. 4003) is explicitly deprecated in this architecture to avoid ambiguity.
- All code logic enforcing this is centralized through the `WS_AUTH_FAILURE_CLOSE_CODE` constant in `config.py`.

## WebSocket Authentication Model

This bridge implements a deterministic, production-ready security policy for WebSocket authentication.

### Threat Model
- **Default (Local)**: The default configuration (`disabled`) assumes a trusted LAN-grade environment where the drone is operated within a private network.
- **Production**: In production, the WebSocket connection MUST be secured using TLS (e.g., `wss://`) via a reverse proxy (like Nginx). **This token-based authentication is not a substitute for TLS.** Without TLS, the token transmitted in the query string is vulnerable to interception via packet sniffing.

### Token Storage & Transmission
- **Storage**: The token is loaded exclusively from the `WS_AUTH_TOKEN` environment variable and is **never** hardcoded.
- **Transmission**: The token is sent by the client via a query parameter:
  `ws://host:port/ws?token=<WS_AUTH_TOKEN>`

### Supported Modes

Auth mode is controlled by `WS_AUTH_MODE` in `config.py` (override via `WS_AUTH_MODE` env var):

| Mode | Behavior | Use Case |
|------|----------|----------|
| **`disabled`** | All connections accepted. No token checked. | Local dev, trusted LAN. **Default.** |
| **`telemetry_only`** | Token absent → accepted, telemetry-only. Token valid → authenticated. Token invalid → rejected (1008). | Mixed LAN with some untoken'd clients. |
| **`required`** | Token missing or wrong → rejected (1008). Token valid → accepted. | Any public or multi-tenant deployment. |

### Token Rotation
Token rotation can be achieved with zero downtime by setting `ALLOW_TOKEN_ROTATION=True` in `config.py`. When enabled, the server dynamically re-evaluates `os.environ["WS_AUTH_TOKEN"]` on every connection attempt. This allows an external orchestration system to update the environment variable securely without requiring a process restart.

### Enforcement Order (required / telemetry_only)

1. Parse `?token=` from query string.
2. Evaluate `_check_ws_auth(token)` → `(allowed, authenticated, reason)`.
3. `ws.accept()` — Starlette requires this before sending a close frame.
4. If **not allowed**: `ws.close(1008, "Policy Violation")` — client gets close frame, RETURN. Client is **never added to registry**.
5. If **allowed**: `clients.add(ws)` — client is now in the registry.
6. In **telemetry_only** mode: unauthenticated clients may receive telemetry but control messages are silently dropped.

### Guarantees

- Unauthenticated clients are **never** added to `clients` (the broadcast registry).
- Token value **never** appears in log output.
- Token comparison is strictly constant-time (`hmac.compare_digest`) to prevent timing attacks.
- Unknown `WS_AUTH_MODE` values fail-closed (reject all connections).
- `/health` exposes `auth.mode`, `auth.token_configured`, and per-client authenticated/unauthenticated counts.
- Close code `1008` = RFC 6455 §7.4.1 Policy Violation (correct for auth failures).

## Process Exit Policy

The bridge follows a **"Fail-Fast"** philosophy. It does not attempt to self-restart internally for core failures. Instead:

1.  If any background task (BLE loop, WebSocket server, Command Consumer) crashes, the entire process **exits with status code 1**.
2.  Recovery is delegated to an **external supervisor** (systemd, Windows Task Scheduler, etc.).

This ensures that partial failures (e.g., the server is down but the BLE loop is alive) never leave the bridge in a "zombie" state.

## Runbook: Running the Bridge

### Development / Direct Execution
Run from the project root:
```powershell
cd apps/aqi-bridge
$env:PYTHONPATH = "."
python -m aqi_bridge
```

### Production: Windows (Task Scheduler)
To ensure the bridge restarts on crash or reboot:
1. Create a "Basic Task" in Task Scheduler.
2. **Trigger**: "At log on" or "At boot".
3. **Action**: "Start a program".
4. **Program**: `C:\Path\To\Python\python.exe`
5. **Arguments**: `-m aqi_bridge`
6. **Start in**: `C:\Users\lev52\Desktop\SASE_AQI_Drone\apps\aqi-bridge`
7. **Settings**: Under "Settings" tab, check "If the task fails, restart every: 1 minute".

### Production: Linux (systemd)
Create `/etc/systemd/system/bridge.service`:
```ini
[Unit]
Description=Drone BLE-WebSocket Bridge
After=network.target

[Service]
ExecStart=/usr/bin/python3 -m aqi_bridge
WorkingDirectory=/opt/SASE_AQI_Drone/apps/aqi-bridge
Restart=always
RestartSec=5
Environment=PYTHONPATH=.
Environment=WS_AUTH_MODE=required
Environment=WS_AUTH_TOKEN=your-secure-token

[Install]
WantedBy=multi-user.target
```

### Production: Nginx Reverse Proxy (TLS Termination)
To ensure secure token transmission, deploy the bridge behind an Nginx reverse proxy with TLS. **Do not expose the Python process directly to the internet if using authenticated control.**

```nginx
server {
    listen 443 ssl;
    server_name drone.example.com;

    ssl_certificate /etc/letsencrypt/live/drone.example.com/fullchain.pem;
    ssl_certificate_key /etc/letsencrypt/live/drone.example.com/privkey.pem;

    location /ws {
        proxy_pass http://127.0.0.1:8765/ws;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "Upgrade";
        proxy_set_header Host $host;
        proxy_set_header X-Forwarded-Proto $scheme;
    }
}
```
*(If setting `ALLOW_TOKEN_ROTATION=True`, the authentication token can be dynamically updated by securely editing the environment dictionary of the running supervisor process without causing drone downtime).*

---

## Telemetry Snapshot Safety

The bridge uses **Option B — Immutable Replacement**. Telemetry objects are never mutated
in-place. Instead, each new reading from the Arduino creates a fresh `TelemetryMessage`
object and atomically replaces `ble.latest_telemetry`.

### Enforcement layers

| Layer | Mechanism | What it prevents |
|-------|-----------|------------------|
| **Pydantic `frozen=True`** | `TelemetryMessage`, `Position`, `GyroReading` all declare `model_config = {"frozen": True}` | Raises `ValidationError` at runtime if any code attempts `telem.aqi = 5` |
| **Assignment-only update path** | `ble.py` assigns `self.latest_telemetry = msg` and never mutates fields | Broadcasts can never see a half-written object |
| **Code comment** | `# IMMUTABLE REPLACEMENT — do NOT mutate in-place` at the assignment site | Makes the rule visible at the point of temptation |
| **Broadcast type assert** | `assert isinstance(telem, TelemetryMessage)` in broadcast loop | Catches wrong-type assignments immediately, before serialization |
| **This section** | Architecture documentation | Developer reference for future refactors |

### Contract

```python
# CORRECT: whole-object replacement (lock held only for this line)
self.latest_telemetry = TelemetryMessage.model_validate(raw)

# FORBIDDEN: raises ValidationError at runtime (frozen=True)
self.latest_telemetry.aqi = 5            # ✗
self.latest_telemetry.position.x = 1.0  # ✗
```

The broadcast loop holds `telemetry_lock` only for the reference copy
(`telem = ble.latest_telemetry`), then releases the lock **before** any
serialization or WebSocket I/O. Because `telem` is frozen, no concurrent
coroutine can alter its fields while serialization is in progress.

> **Note:** Telemetry objects must be **replaced atomically**, not mutated.

---

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

1. All components MUST run entirely within a single `asyncio` event loop.
2. The system MUST prioritize fail-fast reliability over infinite uptime.
3. Network constraints (BLE MTU) define the structural bounds of data flows, not application code.
4. **System-Level bounds MUST be enforced:** The bridge actively monitors memory growth and CPU loop-lag, explicitly failing requests rather than submitting to slow-death memory exhaustion.

## Failure Modes & Mitigation

| Component | Failure | Detection / Consequence | Mitigation Strategy |
|-----------|---------|-------------------------|---------------------|
| **BLE Link** | Disconnect | `disconnected_callback` fires. | Auto-reconnects infinitely with `BLE_RECONNECT_DELAY_S`. |
| **Command Queue** | Oversaturated (PWA sends too fast) | `QueueFull` risk. | `DROP_OLDEST` policy ensures drone always gets newest command. Keeps memory bounded. |
| **Client Connection** | Disconnect / Timeout | `ws.send_text()` deadline exceeded (`WS_SEND_TIMEOUT_S`). | Immediate eviction from registry. One slow client cannot stall the broadcast loop. |
| **Client Connection** | Missing Auth Token | `_check_ws_auth` fails (Mode `required`). | Rejected instantly with WebSocket code `1008` (Policy Violation). |
| **PWA / Network** | App crashes / tab closes | No commands received. | **Deadman Timer:** after `0.5s` of silence, bridge injects `ZERO_COMMAND`. |
| **JSON Payload** | Corrupted / Malformed | JSON decode / Pydantic validation fails. | Packet logged and dropped. Bridge ignores bad data. |
| **JSON Payload** | MTU Size Violation | `len(json) > MAX_TELEMETRY_JSON_BYTES`. | Packet logged and explicitly dropped to prevent buffer overflow. |
| **Telemetry Buffer** | No newlines received | `len(buffer) > MAX_TELEMETRY_BUFFER_SIZE`. | Buffer is aggressively cleared. Fails fast to recover sync. |
| **Event Loop** | CPU Starvation / Blocking Code | Watchdog wakes > `LOOP_STARVATION_THRESHOLD_S` late. | Rate-limited logging; `starvation_count` exposed via `/health`. |
| **System Memory** | Excessive WS Clients | `len(clients) >= MAX_WS_CLIENTS`. | Rejected instantly with WS Close `1013` (Try Again Later) before authentication logic executes. |
| **BLE Write** | OS/Radio Exception during command dispatch | `BleakError` caught by `command_consumer_loop`. | Increments `ble_write.errors` and `dropped`, explicitly calls `ble._safe_disconnect()`. Does **not** retry stale commands. |
| **Bridge Task** | Fatal unhandled exception | Task crashes, event loop continues. | Main Supervisor catches task exit, logs `CRITICAL`, and **terminates the entire Python process (exit 1)** for Docker to restart it. |

## Command Flooding Prevention

```
PWA (burst) → WS → validate → enqueue_command_drop_oldest() → Queue(4) → consumer → BLE write
```

### Drop-Oldest Policy

| Property | Value |
|----------|-------|
| Queue capacity | `COMMAND_QUEUE_SIZE = 4` |
| Overflow policy | `COMMAND_QUEUE_POLICY = "drop_oldest"` |
| Enqueue path | `enqueue_command_drop_oldest()` in `api.py` — **single source of truth** |
| Drop logging | Rate-limited to 1 log/s (`COMMAND_DROP_LOG_INTERVAL_S`) to avoid burst spam |

**Rationale:** the newest command always represents current pilot intent. Forwarding a stale command from 200ms ago is a safety hazard. When overloaded, the bridge discards the oldest (front-of-queue) entry and places the new command at the back.

**Metrics** (visible at `GET /health` under `command_queue`):
- `commands_received` — total calls to `enqueue_command_drop_oldest`
- `commands_enqueued` — total successful `put_nowait` calls
- `commands_dropped_oldest` — total evictions of oldest entries
- `size` — current queue depth

**Guarantee:** `QueueFull` is never raised. `queue.qsize()` never exceeds `COMMAND_QUEUE_SIZE`.

- Only **one** consumer task serialises BLE writes, preventing concurrent GATT operations.

## BLE Write Failure Handling

The bridge architecture inherently uses BLE **write-without-response** to prioritize the lowest possible latency for flight control commands. By eliminating the protocol round-trip for an acknowledgment, the control loop stays tight, but it introduces the risk of silent delivery failures if the underlying radio stack or OS drops the packet mid-flight.

To guarantee macroscopic determinism when a write explicitly fails:

1. **Propagate, Don't Swallow:** `_write_command_bytes` no longer catches `BleakError`. It decrements `writes_in_flight` and raises the exception up to the consumer loop.
2. **Deterministic Drop Policy:** Stale control commands are **never** retried. An old command is dangerous. Dropping it ensures only fresh commands dictate pilot intent.
3. **Fail-Fast Deflection:** If an exception is caught, the consumer loop explicitly triggers an `await ble._safe_disconnect()`. This forces the BLE stack out of a potentially zombie "connected-but-failing" state, transitioning the internal system state to `DEGRADED`, and restarting the scan-and-reconnect workflow cleanly.
4. **Observable Metrics:** The bridge maintains a deterministic scorecard exposed under `GET /health` (`ble_write.errors`, `dropped`, `last_error_timestamp`).

> **Debugging Downgrade:** If required for debugging radio instability, modify `config.py` to set `USE_WRITE_WITH_RESPONSE = True`. This will force the MAC layer to wait for GATT acknowledgments at the severe cost of latency.
- BLE write-without-response is used to minimise round-trip latency.

## Deployment Hygiene

- **Reverse Proxy**: For production, layer TLS (wss://) via a reverse proxy (**nginx** or **Caddy**).
- **Network Boundaries**: The bridge binds to `0.0.0.0:8765` by default; ensure your firewall restricts access if not using the token auth.
- **Monitoring**: Use the `/health` endpoint with a watchdog (like UptimeRobot or a local cron script) to alert on bridge downtime.

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
cd C:\Users\lev52\Desktop\SASE_AQI_Drone\apps\aqi-bridge
$env:PYTHONPATH = "."
& 'C:\Users\lev52\AppData\Local\Programs\Python\Python313\python.exe' -m aqi_bridge
```

The server starts on `http://0.0.0.0:8765`. Health check: `GET /health`. WebSocket: `ws://localhost:8765/ws`.
