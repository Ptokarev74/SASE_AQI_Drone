"""
FastAPI WebSocket server that bridges PWA clients to the BLE drone.

Responsibilities
-----------------
* ``/ws`` — WebSocket endpoint: receives control commands, broadcasts telemetry.
* ``/health`` — REST health-check returning BLE status and client count.
* Manages a client registry (``set`` of WebSocket connections).
* Runs a telemetry broadcast loop at a configurable rate.
* Pushes validated control commands into a bounded ``asyncio.Queue``.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from typing import TYPE_CHECKING, Set

from fastapi import FastAPI, Query, WebSocket, WebSocketDisconnect
from starlette.websockets import WebSocketState

from bridge.config import (
    COMMAND_QUEUE_SIZE,
    TELEMETRY_BROADCAST_HZ,
    WS_AUTH_TOKEN,
)
from bridge.schemas import ControlCommand

if TYPE_CHECKING:
    from bridge.ble_client import BLEDroneClient

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Application factory
# ---------------------------------------------------------------------------

def create_app(
    ble: "BLEDroneClient",
    command_queue: asyncio.Queue[ControlCommand],
) -> FastAPI:
    """
    Build and return the FastAPI application.

    Parameters
    ----------
    ble:
        Shared BLE client instance (for reading telemetry + connection state).
    command_queue:
        Bounded queue that feeds commands to the BLE write consumer.
    """

    app = FastAPI(title="AQI Drone Bridge", version="1.0.0")

    # --- client registry ------------------------------------------------
    clients: Set[WebSocket] = set()

    # ----------------------------------------------------------------
    # Health endpoint
    # ----------------------------------------------------------------
    @app.get("/health")
    async def health() -> dict:
        return {
            "ble_connected": ble.connected.is_set(),
            "ws_clients": len(clients),
        }

    # ----------------------------------------------------------------
    # WebSocket endpoint
    # ----------------------------------------------------------------
    @app.websocket("/ws")
    async def websocket_endpoint(
        ws: WebSocket,
        token: str | None = Query(default=None),
    ) -> None:
        # --- optional token auth ----------------------------------------
        if WS_AUTH_TOKEN is not None:
            if token != WS_AUTH_TOKEN:
                await ws.close(code=4003, reason="Forbidden")
                return

        await ws.accept()
        clients.add(ws)
        logger.info("WS client connected (%s) — total %d", ws.client, len(clients))

        try:
            while True:
                raw = await ws.receive_text()
                _handle_control_message(raw, command_queue)
        except WebSocketDisconnect:
            pass
        except Exception:
            logger.exception("WS client error")
        finally:
            clients.discard(ws)
            logger.info("WS client disconnected — total %d", len(clients))

    # ----------------------------------------------------------------
    # Attach helpers as app state so main.py can access them
    # ----------------------------------------------------------------
    app.state.clients = clients  # type: ignore[attr-defined]
    app.state.ble = ble          # type: ignore[attr-defined]

    return app


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _handle_control_message(
    raw: str,
    queue: asyncio.Queue[ControlCommand],
) -> None:
    """Validate incoming JSON and enqueue.  Drop on full queue (backpressure)."""
    try:
        data = json.loads(raw)
        cmd = ControlCommand.model_validate(data)
    except Exception as exc:
        logger.debug("Bad control message: %s — %s", exc, raw[:200])
        return

    # Drop-oldest policy: if queue is full, pop the stale front entry.
    while queue.full():
        try:
            queue.get_nowait()
        except asyncio.QueueEmpty:
            break
    try:
        queue.put_nowait(cmd)
    except asyncio.QueueFull:
        pass  # truly full — extremely unlikely after drain above


async def broadcast_telemetry_loop(
    ble: "BLEDroneClient",
    clients: Set[WebSocket],
) -> None:
    """
    Periodically read the latest telemetry snapshot and push to every
    connected WebSocket client.

    Runs at ``TELEMETRY_BROADCAST_HZ`` Hz.  Clients that error during
    send are silently removed from the registry.
    """
    interval = 1.0 / TELEMETRY_BROADCAST_HZ
    last_sent: str | None = None  # avoid re-sending identical frames

    while True:
        try:
            await asyncio.sleep(interval)

            telem = ble.latest_telemetry
            if telem is None:
                continue

            payload = telem.model_dump_json()

            # Skip if telemetry hasn't changed (saves bandwidth)
            if payload == last_sent:
                continue
            last_sent = payload

            # Fan-out to all clients; collect dead sockets to prune
            dead: list[WebSocket] = []
            for ws in list(clients):
                try:
                    if ws.client_state == WebSocketState.CONNECTED:
                        await ws.send_text(payload)
                except Exception:
                    dead.append(ws)

            for ws in dead:
                clients.discard(ws)
                logger.info("Pruned dead WS client — total %d", len(clients))

        except asyncio.CancelledError:
            logger.info("Telemetry broadcast loop cancelled")
            raise
        except Exception:
            logger.exception("Error in broadcast loop")
            await asyncio.sleep(1)
