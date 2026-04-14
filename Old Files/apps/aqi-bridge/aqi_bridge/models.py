"""
Message schemas for telemetry and control.

Uses Pydantic v2 for fast validation and JSON (de)serialisation.
Fields are kept optional where the Arduino may not yet report all sensors,
so the bridge never crashes on a missing key.

Immutability contract
----------------------
``TelemetryMessage`` (and its sub-models) are declared ``frozen=True``.
This structurally prevents in-place field mutation.  Callers MUST replace
``ble.latest_telemetry`` with a brand-new object rather than mutating
existing fields.  See ARCHITECTURE.md "Telemetry Snapshot Safety".
"""

from __future__ import annotations

import struct
import zlib

from pydantic import BaseModel, ConfigDict, Field


# ---------------------------------------------------------------------------
# Telemetry  (Arduino → Bridge → PWA)
# ---------------------------------------------------------------------------
class Position(BaseModel):
    """Frozen: do not mutate fields in-place. Replace the parent TelemetryMessage instead."""
    model_config = {"frozen": True}

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class GyroReading(BaseModel):
    """Frozen: do not mutate fields in-place. Replace the parent TelemetryMessage instead."""
    model_config = {"frozen": True}

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class TelemetryMessage(BaseModel):
    """
    Telemetry payload received from the Arduino over BLE notifications.

    All fields are optional with defaults so partial payloads are tolerated.

    Immutability contract
    ----------------------
    ``frozen=True`` prevents any code from mutating fields in-place (e.g.,
    ``telem.aqi = 5`` raises a ``ValidationError`` at runtime).  To update
    telemetry, create a NEW ``TelemetryMessage`` and assign it atomically:

        # CORRECT: full replacement
        self.latest_telemetry = TelemetryMessage.model_validate(raw)

        # FORBIDDEN: in-place mutation — will raise at runtime
        self.latest_telemetry.aqi = 5  # ✗ raises ValidationError
    """
    model_config = {"frozen": True}

    status: str = "unknown"
    position: Position = Field(default_factory=Position)
    gyro: GyroReading = Field(default_factory=GyroReading)

    # Air quality — ENS160
    aqi: int | None = None
    tvoc: int | None = None       # ppb
    eco2: int | None = None       # ppm

    # Environment — AHT20
    temperature_c: float | None = None
    humidity_pct: float | None = None

    # Millisecond timestamp from the Arduino (if provided)
    timestamp_ms: int | None = None


# ---------------------------------------------------------------------------
# Control commands  (PWA → Bridge → Arduino)
# ---------------------------------------------------------------------------
class ControlCommand(BaseModel):
    """
    Commands sent from the web client, destined for the drone firmware.
    Contains strict float domains to inherently reject pathological scaling 
    or NaN/Infinity exploits before hitting the BLE transceiver.
    """

    model_config = ConfigDict(extra="ignore")

    vx: float = Field(default=0.0, ge=-10000.0, le=10000.0, allow_inf_nan=False)
    vy: float = Field(default=0.0, ge=-10000.0, le=10000.0, allow_inf_nan=False)
    vz: float = Field(default=0.0, ge=-10000.0, le=10000.0, allow_inf_nan=False)
    yaw: float = Field(default=0.0, ge=-10000.0, le=10000.0, allow_inf_nan=False)
    arm: bool = False

    # Performance instrumentation (monotonic timestamps)
    ts_received: float = 0.0
    ts_enqueued: float = 0.0
    ts_dequeued: float = 0.0
    ts_write_started: float = 0.0
    ts_dispatched: float = 0.0

    def pack_binary(self) -> bytes:
        """
        Pack the command into a 21-byte binary packet:
        [vx: f32][vy: f32][vz: f32][yaw: f32][arm: u8][crc32: u32]
        Use little-endian ('<') for deterministic Arduino compatibility.
        """
        # 1. Pack the core fields (17 bytes: 4x floats + 1x bool/u8)
        packed_data = struct.pack(
            "<ffffB",
            self.vx,
            self.vy,
            self.vz,
            self.yaw,
            1 if self.arm else 0
        )
        
        # 2. Calculate CRC32 over the 17 bytes
        crc = zlib.crc32(packed_data) & 0xFFFFFFFF
        
        # 3. Append the CRC32 (4 bytes)
        return packed_data + struct.pack("<I", crc)

    # Safety instrumentation: marks autonomous bridge-injected overrides
    is_failsafe: bool = False


# Pre-built constant: The explicitly defined safety override command.
# Bounding invariants dictate this MUST arm=False and zeroes all velocity.
FAILSAFE_COMMAND = ControlCommand(
    vx=0.0, vy=0.0, vz=0.0, yaw=0.0, arm=False, is_failsafe=True
)
