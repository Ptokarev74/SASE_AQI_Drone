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

from pydantic import BaseModel, Field


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
    Velocity + yaw + arm command sent from the PWA controller.

    All velocity values are in the drone's body frame.
    """

    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw: float = 0.0
    arm: bool = False


# Pre-built constant: the command that stops all motion / disarms.
ZERO_COMMAND = ControlCommand(vx=0.0, vy=0.0, vz=0.0, yaw=0.0, arm=False)
