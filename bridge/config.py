"""
Centralized configuration for the BLE-to-WebSocket bridge.

All BLE UUIDs, network settings, timing parameters, and queue sizes
are defined here so the rest of the codebase never hard-codes magic values.
"""
import os as _os

# ---------------------------------------------------------------------------
# BLE — Arduino peripheral identifiers
# ---------------------------------------------------------------------------
# Change these to match your Arduino firmware's advertised service/char UUIDs.
BLE_DEVICE_NAME: str = "AQI_Drone"

# UUID of the BLE characteristic the Arduino uses to NOTIFY telemetry JSON.
TELEMETRY_CHAR_UUID: str = "19b10011-e8f2-537e-4f6c-d104768a1214"

# UUID of the BLE characteristic the Arduino accepts WRITE commands on.
COMMAND_CHAR_UUID: str = "19b10012-e8f2-537e-4f6c-d104768a1214"

# Seconds to wait before retrying a BLE connection after disconnect.
BLE_RECONNECT_DELAY_S: float = 3.0

# Seconds to wait for a BLE scan to find the device before retrying.
BLE_SCAN_TIMEOUT_S: float = 10.0

# ---------------------------------------------------------------------------
# BLE MTU
# ---------------------------------------------------------------------------
# ArduinoBLE default ATT MTU is 23 bytes → max notification payload = 20 bytes.
# Bleak will attempt to negotiate a higher MTU (up to 247) automatically on
# many platforms.  The bridge logs the actual negotiated MTU on every connect.
#
# BLE_TARGET_MTU_BYTES: the MTU we want bleak to negotiate with the peripheral.
# BLE_OVERHEAD_BYTES:   ATT notification header overhead (3 bytes: opcode + handle).
# After negotiation: usable_payload = negotiated_mtu - BLE_OVERHEAD_BYTES.
BLE_TARGET_MTU_BYTES: int = 247          # max allowed by Bluetooth 4.2+
BLE_OVERHEAD_BYTES: int = 3              # ATT layer overhead
BLE_DEFAULT_MTU_BYTES: int = 23         # BLE spec default
BLE_DEFAULT_PAYLOAD_BYTES: int = BLE_DEFAULT_MTU_BYTES - BLE_OVERHEAD_BYTES  # = 20

# If the negotiated payload is below this threshold, chunked framing is enabled.
BLE_MIN_DIRECT_PAYLOAD_BYTES: int = 185  # ~(247 - 3) * 0.75 — safe threshold

# ---------------------------------------------------------------------------
# Chunked framing protocol
# ---------------------------------------------------------------------------
# Header format (all ASCII, pipe-delimited) prepended to each chunk:
#   <seq_id>|<total_chunks>|<chunk_index>|<data>\n
# seq_id:        0-65535, wraps around
# total_chunks:  1-255
# chunk_index:   0-based
#
# Maximum assembled message size and per-sequence assembly timeout.
CHUNK_MAX_MESSAGE_SIZE: int = 2048       # bytes; guards against runaway assemblies
CHUNK_ASSEMBLY_TIMEOUT_S: float = 2.0   # discard incomplete sequences older than this

# ---------------------------------------------------------------------------
# Command pipeline
# ---------------------------------------------------------------------------
# Max items in the command queue.  When full, the *oldest* command is dropped
# so the drone always receives the most recent pilot intent.
COMMAND_QUEUE_SIZE: int = 4

# Overflow policy (informational constant — enforced by enqueue_command_drop_oldest).
# "drop_oldest": discard the front of the queue (oldest) and enqueue the new command.
COMMAND_QUEUE_POLICY: str = "drop_oldest"

# Rate limit for debug logs when dropping stale commands.
# Prevents I/O spam during high-load burst periods.
COMMAND_DROP_LOG_INTERVAL_S: float = 1.0

# ---------------------------------------------------------------------------
# Deadman safety timer
# ---------------------------------------------------------------------------
# TRADEOFF: Safety Stop > Jitter Tolerance
# Maximum time without a control command from the PWA before the bridge
# injects a ZERO_COMMAND (stop all motors). 0.5s is aggressive; it may cause
# stuttering on poor networks, but strictly prevents "flyaways" if the
# mobile connection drops.
DEADMAN_TIMEOUT_S: float = 0.5

# ---------------------------------------------------------------------------
# Telemetry broadcast
# ---------------------------------------------------------------------------
# Maximum rate (Hz) at which telemetry snapshots are pushed to WS clients.
TELEMETRY_BROADCAST_HZ: float = 30.0

# ---------------------------------------------------------------------------
# Telemetry Reassembly (Hardening)
# ---------------------------------------------------------------------------
# Every JSON message MUST be terminated with '\n'.
# Payload itself must not contain newlines.
MAX_TELEMETRY_MESSAGE_SIZE: int = 1024
MAX_TELEMETRY_BUFFER_SIZE: int = 4096

# Absolute limit for a fully reassembled JSON string.
# If an Arduino payload (direct or reassembled from chunks) exceeds this,
# it is explicitly dropped to prevent memory exhaustion / MTU overflow.
MAX_TELEMETRY_JSON_BYTES: int = 1024

# ---------------------------------------------------------------------------
# BLE communication settings
# ---------------------------------------------------------------------------
BLE_RECONNECT_DELAY_S: float = 2.0
BLE_CONNECTION_TIMEOUT_S: float = 10.0

# Protocol negotiation timeout
CHUNK_MTU_NEGOTIATION_TIMEOUT_S: float = 2.0

# Control command write mode toggle. Defaults to False for minimum latency.
# If True, commands execute using Write With Response to guarantee acknowledgment,
# at the cost of higher latency per command.
USE_WRITE_WITH_RESPONSE: bool = False

# ---------------------------------------------------------------------------
# WebSocket server
# ---------------------------------------------------------------------------
WS_HOST: str = "0.0.0.0"
WS_PORT: int = 8765

# Per-client send deadline (seconds).  If ws.send_text() doesn't complete
# within this window, the client is assumed dead and immediately removed.
# Prevents a single slow/stalled client from blocking the entire broadcast.
WS_SEND_TIMEOUT_S: float = 0.1  # 100 ms

# How often (seconds) the broadcast loop logs a performance summary line:
#   "Broadcast cycle: N clients, M dropped, X.Xms"
WS_BROADCAST_LOG_INTERVAL_S: float = 10.0

# ---------------------------------------------------------------------------
# System-Level Failure Mitigation & Memory Bounds
# ---------------------------------------------------------------------------
# Unbounded client growth causes OOM and CPU starvation.
MAX_WS_CLIENTS: int = 100

# Event loop watchdog: if the actual wake delay exceeds this threshold (seconds),
# the loop is considered starved (e.g. by synchronous CPU-bound operations or floods).
LOOP_STARVATION_THRESHOLD_S: float = 0.1

# Minimum time between identical high-frequency log warnings to prevent I/O spam.
LOG_RATE_LIMIT_S: float = 1.0

# ---------------------------------------------------------------------------
# WebSocket authentication policy
# ---------------------------------------------------------------------------
# Three explicit modes:
#
#   "disabled"       — No auth checks. All connections are accepted.
#                      Use for local development / trusted LAN.
#                      DEFAULT (secure-by-default would be "required").
#
#   "telemetry_only" — If ?token= is present it MUST be valid; if absent the
#                      connection is still accepted but the client is marked
#                      unauthenticated and may NOT send control commands.
#                      Use for mixed LAN environments where some clients lack tokens.
#
#   "required"       — Token MUST be present and valid or connection is rejected
#                      with WebSocket close code 1008 (Policy Violation).
#                      Use for any public-facing or multi-tenant deployment.
#
# Override via environment variable WS_AUTH_MODE for production deployments.
WS_AUTH_MODE: str = _os.environ.get("WS_AUTH_MODE", "disabled")

# The shared-secret token.  Empty string / unset = no token configured.
# In "required" mode a missing WS_AUTH_TOKEN constant is itself a misconfiguration
# and will be caught at startup (server.py logs a WARNING).
# Set via environment: export WS_AUTH_TOKEN="your-secret-here"
WS_AUTH_TOKEN: str = _os.environ.get("WS_AUTH_TOKEN", "")

# Query-param name clients use to pass the token:
#   ws://host:port/ws?<WS_AUTH_QUERY_PARAM>=<token>
WS_AUTH_QUERY_PARAM: str = "token"

# Determines if the server should dynamically reload the WS_AUTH_TOKEN
# from the environment on every authentication check. Set to True to enable
# zero-downtime token rotation. Default is False (token is loaded once at startup).
ALLOW_TOKEN_ROTATION: bool = _os.environ.get("ALLOW_TOKEN_ROTATION", "false").lower() == "true"

# WebSocket close code sent to rejected clients.
# 1008 = Policy Violation (RFC 6455 §7.4.1) — correct close code for auth failures.
WS_AUTH_FAILURE_CLOSE_CODE: int = 1008

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
# Supported formats: "text", "json"
LOG_FORMAT: str = _os.environ.get("LOG_FORMAT", "text").lower()
LOG_LEVEL: str = _os.environ.get("LOG_LEVEL", "INFO").upper()

# ---------------------------------------------------------------------------
# Debugging / Reliability testing
# ---------------------------------------------------------------------------
# If True, the POST /debug/crash endpoint is enabled to help test supervisor
# restart behavior.  KEEP AS FALSE IN PRODUCTION.
ALLOW_DEBUG_CRASH: bool = _os.environ.get("ALLOW_DEBUG_CRASH", "0") == "1"

del _os  # clean up module namespace
