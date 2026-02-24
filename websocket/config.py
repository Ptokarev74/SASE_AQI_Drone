"""
Centralized configuration for the BLE-to-WebSocket bridge.

All BLE UUIDs, network settings, timing parameters, and queue sizes
are defined here so the rest of the codebase never hard-codes magic values.
"""

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
# Command pipeline
# ---------------------------------------------------------------------------
# Max items in the command queue.  When full, the *oldest* command is dropped
# so the drone always receives the most recent pilot intent.
COMMAND_QUEUE_SIZE: int = 4

# ---------------------------------------------------------------------------
# Deadman safety
# ---------------------------------------------------------------------------
# If no control command arrives within this window, automatically send a
# zero-velocity stop command to the drone.
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

# ---------------------------------------------------------------------------
# WebSocket server
# ---------------------------------------------------------------------------
WS_HOST: str = "0.0.0.0"
WS_PORT: int = 8765

# Optional bearer token for simple WebSocket auth.
# Set to None to disable auth.  Clients pass it as a query param:
#   ws://host:port/ws?token=<TOKEN>
WS_AUTH_TOKEN: str | None = None
