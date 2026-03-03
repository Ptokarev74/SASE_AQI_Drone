# AQI Drone Bridge — Protocol Specification

This document defines the application-layer and link-layer protocols used between the Arduino drone peripheral and the Python WebSocket bridge.

## 1. BLE Telemetry Protocol (Peripheral → Bridge)

The Arduino transmits telemetry as JSON over a BLE Notify characteristic. Due to BLE MTU limitations, the bridge supports two operational modes depending on the negotiated MTU.

### Mode A: Direct NDJSON (When MTU allows Payload ≥ 185 bytes)

When the negotiated BLE connection allows a usable payload large enough to fit an entire JSON string, the system operates in **Direct Mode** using NDJSON (Newline-Delimited JSON).

**Format:**
```json
{"status": "flying", "aqi": 45, "tvoc": 120, "eco2": 400}\n
```

**Rules:**
- Every payload **MUST** end with a newline `\n`.
- The JSON payload itself must NOT contain unescaped newline characters.
- Maximum total length (including `\n`) is defined by `MAX_TELEMETRY_JSON_BYTES` (default 1024 bytes).

### Mode B: Chunked Framing Protocol (When MTU Payload < 185 bytes)

When the connection MTU is insufficient (e.g., the default 23-byte MTU allowing only 20 bytes of payload), the Arduino must split the JSON string into smaller chunks. The bridge enters **Chunked Mode** and reconstructs them.

**Format (Pipe-Delimited Header):**
`<seq_id>|<total_chunks>|<chunk_index>|<data>\n`

**Fields:**
- `seq_id`: An integer identifying the assembly (e.g. `0` to `65535`).
- `total_chunks`: The total number of chunks making up the full JSON string.
- `chunk_index`: The 0-based index of the current chunk.
- `data`: The partial string of the JSON payload.
- `\n`: A newline character terminating the BLE notification.

**Example (A JSON string split into 3 chunks):**
```text
42|3|0|{"status":"flying"
42|3|1|,"aqi":45,"tvoc":12
42|3|2|0,"eco2":400}
```
*Note: The newline `\n` is physically present at the end of every BLE transmitted chunk frame to fulfill Layer 1 byte-buffer splitting, though it is not part of the final reassembled JSON string.*

**Assembly Rules:**
- The bridge collects chunks with identical `seq_id`.
- Once all `total_chunks` are received, the `data` segments are concatenated in `chunk_index` order.
- The resulting string must be valid JSON and falls under the `MAX_TELEMETRY_JSON_BYTES` enforcement limit.
- If chunks for a given `seq_id` take longer than `CHUNK_ASSEMBLY_TIMEOUT_S` (2.0s) to arrive, the incomplete assembly is discarded.

## 2. WebSocket Control Protocol (PWA → Bridge)

The PWA sends flight commands to the Python bridge over WebSocket. The bridge parses the JSON, validates it against the `ControlCommand` Pydantic model (`aqi_bridge/models.py`), and serializes it into a binary packet sent over BLE to the Arduino.

**Format:**
```json
{
  "vx": 0.0,
  "vy": 0.0,
  "vz": 0.0,
  "yaw": 0.0,
  "arm": false
}
```

**Fields:**
| Field | Type    | Range               | Description                                             |
|-------|---------|---------------------|---------------------------------------------------------|
| `vx`  | float   | `-10000.0..10000.0` | Forward/backward velocity. Positive = forward.          |
| `vy`  | float   | `-10000.0..10000.0` | Left/right strafe. Positive = right.                    |
| `vz`  | float   | `-10000.0..10000.0` | Vertical velocity. Positive = up.                       |
| `yaw` | float   | `-10000.0..10000.0` | Yaw rate. Positive = clockwise (viewed from above).     |
| `arm` | boolean | `true / false`      | Motor arm state. Must be `true` for the drone to move.  |

**Wire Format (BLE):**
The bridge serializes validated commands into a 21-byte binary packet before writing to `COMMAND_CHAR_UUID`:
```
[vx: f32][vy: f32][vz: f32][yaw: f32][arm: u8][crc32: u32]  (little-endian)
```

**Safety Rules:**
- Commands are subject to a **Drop-Oldest Queue Policy**: if the queue is full, the oldest command is evicted to preserve freshness.
- A **Deadman Timer** fires if no command is received within 0.5 s — the bridge auto-enqueues a zero/disarm failsafe and locks out further input until an explicit `arm: true` re-arm is sent.
- Commands older than `MAX_COMMAND_AGE_MS` (300 ms) are silently dropped at dequeue time.
