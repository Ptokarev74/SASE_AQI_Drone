"""
Async BLE central that connects to the Arduino drone peripheral.

Responsibilities
-----------------
* Scan for the device by advertised name.
* Connect and subscribe to telemetry notifications.
* Parse incoming JSON telemetry, validate with Pydantic, store atomic snapshot.
* Expose ``write_command`` for writing control JSON to the command characteristic.
* Auto-reconnect forever on BLE disconnect.

Thread safety: ``latest_telemetry`` is guarded by an ``asyncio.Lock`` so the
broadcast loop in the server module can read it concurrently.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from typing import Optional

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

from bridge.config import (
    BLE_DEVICE_NAME,
    BLE_RECONNECT_DELAY_S,
    BLE_SCAN_TIMEOUT_S,
    COMMAND_CHAR_UUID,
    TELEMETRY_CHAR_UUID,
)
from bridge.schemas import ControlCommand, TelemetryMessage

logger = logging.getLogger(__name__)


class BLEDroneClient:
    """Manages the BLE connection to a single Arduino drone peripheral."""

    def __init__(self) -> None:
        # --- public state (read by other modules) --------------------------
        self.connected: asyncio.Event = asyncio.Event()
        self.latest_telemetry: Optional[TelemetryMessage] = None
        self.telemetry_lock: asyncio.Lock = asyncio.Lock()

        # --- internal -------------------------------------------------------
        self._client: Optional[BleakClient] = None
        self._buffer: bytes = b""  # Byte-level reassembly buffer for partial JSON

    # ------------------------------------------------------------------
    # Scanning & connection
    # ------------------------------------------------------------------
    async def _scan(self) -> Optional[str]:
        """Scan for the drone by advertised name.  Returns the BLE address or None."""
        logger.info("Scanning for BLE device '%s' (timeout %ss) ...", BLE_DEVICE_NAME, BLE_SCAN_TIMEOUT_S)
        devices = await BleakScanner.discover(timeout=BLE_SCAN_TIMEOUT_S)
        for d in devices:
            if d.name and BLE_DEVICE_NAME.lower() in d.name.lower():
                logger.info("Found device: %s [%s]", d.name, d.address)
                return d.address
        logger.warning("Device '%s' not found in scan.", BLE_DEVICE_NAME)
        return None

    async def _connect(self, address: str) -> None:
        """Connect to the given BLE address, subscribe to telemetry notifications."""
        logger.info("Connecting to %s ...", address)
        self._client = BleakClient(
            address,
            disconnected_callback=self._on_disconnect,
        )
        await self._client.connect()
        logger.info("Connected to %s", address)

        # Subscribe to telemetry notifications
        await self._client.start_notify(
            TELEMETRY_CHAR_UUID,
            self._on_telemetry_notification,
        )
        logger.info("Subscribed to telemetry notifications (%s)", TELEMETRY_CHAR_UUID)
        self.connected.set()

    # ------------------------------------------------------------------
    # Notification handler
    # ------------------------------------------------------------------
    def _on_telemetry_notification(self, _sender: int, data: bytearray) -> None:
        """
        Called by bleak on every BLE notification.

        Data may arrive fragmented (BLE MTU limits), so we accumulate in
        ``self._buffer`` and attempt JSON parsing only when we see a
        complete object (newline-delimited).
        """
        # 1. Byte-level append
        self._buffer += data

        # 2. Extract complete frames delimited by b'\n'
        while b"\n" in self._buffer:
            line, self._buffer = self._buffer.split(b"\n", 1)
            line = line.strip()
            if not line:
                continue

            # 3. Process frame
            self._process_frame(line)

        # 4. Overflow guard: prevent memory runaway if no newline arrives
        from bridge.config import MAX_TELEMETRY_BUFFER_SIZE
        if len(self._buffer) > MAX_TELEMETRY_BUFFER_SIZE:
            logger.warning(
                "Telemetry buffer overflow (%d bytes) — no newline detected. Resetting.",
                len(self._buffer)
            )
            self._buffer = b""

    def _process_frame(self, line_bytes: bytes) -> None:
        """Decode, parse, and validate a single telemetry frame."""
        # Check size before expensive decoding
        from bridge.config import MAX_TELEMETRY_MESSAGE_SIZE
        if len(line_bytes) > MAX_TELEMETRY_MESSAGE_SIZE:
            logger.warning("Telemetry frame too large (%d bytes). Discarding.", len(line_bytes))
            return

        # Decode UTF-8 safely
        try:
            line = line_bytes.decode("utf-8")
        except UnicodeDecodeError as exc:
            logger.error("Failed to decode telemetry as UTF-8: %s", exc)
            return

        # Parse JSON
        try:
            raw = json.loads(line)
        except json.JSONDecodeError as exc:
            logger.debug("Malformed JSON telemetry (discarded): %s — %s", exc, line[:80])
            return

        # Validate with Pydantic
        try:
            msg = TelemetryMessage.model_validate(raw)
        except Exception as exc:  # noqa: BLE001
            logger.warning("Telemetry validation failed (discarded): %s", exc)
            return

        # Store atomic snapshot
        self.latest_telemetry = msg
        logger.debug("Received valid telemetry frame (%d bytes)", len(line_bytes))

    # ------------------------------------------------------------------
    # Command writer
    # ------------------------------------------------------------------
    async def write_command(self, cmd: ControlCommand) -> bool:
        """
        Serialise *cmd* to JSON bytes and write to the command characteristic.

        Returns ``True`` on success, ``False`` if the BLE link is down.
        Uses write-without-response for lowest latency.
        """
        if self._client is None or not self._client.is_connected:
            return False

        payload = cmd.model_dump_json().encode("utf-8")
        try:
            await self._client.write_gatt_char(
                COMMAND_CHAR_UUID,
                payload,
                response=False,  # write-without-response for speed
            )
            return True
        except BleakError as exc:
            logger.error("BLE write failed: %s", exc)
            return False

    # ------------------------------------------------------------------
    # Disconnect callback
    # ------------------------------------------------------------------
    def _on_disconnect(self, _client: BleakClient) -> None:
        logger.warning("BLE disconnected.")
        self.connected.clear()
        self._telemetry_buffer = ""

    # ------------------------------------------------------------------
    # Reconnection loop — the public entry point
    # ------------------------------------------------------------------
    async def run_forever(self) -> None:
        """
        Outer loop: scan → connect → wait for disconnect → sleep → retry.

        This coroutine never returns under normal operation.
        """
        while True:
            try:
                address = await self._scan()
                if address is None:
                    logger.info("Retrying scan in %ss ...", BLE_RECONNECT_DELAY_S)
                    await asyncio.sleep(BLE_RECONNECT_DELAY_S)
                    continue

                await self._connect(address)

                # Block here until disconnected
                while self._client and self._client.is_connected:
                    await asyncio.sleep(0.5)

            except BleakError as exc:
                logger.error("BLE error: %s", exc)
            except asyncio.CancelledError:
                logger.info("BLE client task cancelled — disconnecting")
                await self._safe_disconnect()
                raise
            except Exception:
                logger.exception("Unexpected error in BLE loop")

            # Clean up before retrying
            await self._safe_disconnect()
            logger.info("Reconnecting in %ss ...", BLE_RECONNECT_DELAY_S)
            await asyncio.sleep(BLE_RECONNECT_DELAY_S)

    async def _safe_disconnect(self) -> None:
        """Gracefully close the BLE connection if still open."""
        self.connected.clear()
        if self._client is not None:
            try:
                if self._client.is_connected:
                    await self._client.disconnect()
            except Exception:
                pass
            self._client = None
