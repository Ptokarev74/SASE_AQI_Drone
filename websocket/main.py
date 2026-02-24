"""
Orchestrator — ties BLE client, WebSocket server, deadman timer, and
command consumer together under a single asyncio event loop.

Run with:
    python -m bridge.main
"""

from __future__ import annotations

import asyncio
import logging
import signal
import sys
import time

import uvicorn

from bridge.ble_client import BLEDroneClient
from bridge.config import (
    COMMAND_QUEUE_SIZE,
    DEADMAN_TIMEOUT_S,
    WS_HOST,
    WS_PORT,
)
from bridge.schemas import ControlCommand, ZERO_COMMAND
from bridge.server import broadcast_telemetry_loop, create_app

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(name)s  %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("bridge")


# ---------------------------------------------------------------------------
# Command consumer — pops from queue → writes to BLE
# ---------------------------------------------------------------------------
async def command_consumer(
    ble: BLEDroneClient,
    queue: asyncio.Queue[ControlCommand],
    last_cmd_time: list[float],
) -> None:
    """
    Drains the command queue and writes each to the BLE command characteristic.

    Updates *last_cmd_time[0]* on every successful write so the deadman timer
    knows when the last real command arrived.
    """
    while True:
        cmd = await queue.get()
        ok = await ble.write_command(cmd)
        if ok:
            last_cmd_time[0] = time.monotonic()


# ---------------------------------------------------------------------------
# Deadman timer — send ZERO_COMMAND if pilot goes silent
# ---------------------------------------------------------------------------
async def deadman_timer(
    ble: BLEDroneClient,
    last_cmd_time: list[float],
) -> None:
    """
    If no control command has been received for ``DEADMAN_TIMEOUT_S`` seconds,
    send a zero-velocity / disarm command to the drone.

    This is critical safety logic: if the WebSocket client crashes, the phone
    loses connectivity, or the PWA tab is closed, the drone should stop.
    """
    fired = False  # avoid spamming zero commands

    while True:
        await asyncio.sleep(0.1)  # check at 10 Hz

        # Only enforce deadman when BLE is connected and we've ever received
        # at least one command (last_cmd_time != 0).
        if not ble.connected.is_set() or last_cmd_time[0] == 0.0:
            fired = False
            continue

        elapsed = time.monotonic() - last_cmd_time[0]

        if elapsed > DEADMAN_TIMEOUT_S:
            if not fired:
                logger.warning(
                    "DEADMAN: no command for %.2fs — sending ZERO_COMMAND",
                    elapsed,
                )
                await ble.write_command(ZERO_COMMAND)
                fired = True
        else:
            fired = False


# ---------------------------------------------------------------------------
# Uvicorn in-process runner
# ---------------------------------------------------------------------------
async def run_uvicorn(app: object) -> None:
    """Run Uvicorn as an async task so it lives inside our event loop."""
    config = uvicorn.Config(
        app,
        host=WS_HOST,
        port=WS_PORT,
        log_level="info",
        ws="websockets",
    )
    server = uvicorn.Server(config)
    await server.serve()


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------
async def _run() -> None:
    # --- shared state ---------------------------------------------------
    ble = BLEDroneClient()
    command_queue: asyncio.Queue[ControlCommand] = asyncio.Queue(
        maxsize=COMMAND_QUEUE_SIZE,
    )
    # Mutable container so coroutines can update by reference.
    last_cmd_time: list[float] = [0.0]

    app = create_app(ble, command_queue)

    # --- spawn concurrent tasks -----------------------------------------
    tasks = [
        asyncio.create_task(ble.run_forever(), name="ble_loop"),
        asyncio.create_task(run_uvicorn(app), name="uvicorn"),
        asyncio.create_task(
            command_consumer(ble, command_queue, last_cmd_time),
            name="cmd_consumer",
        ),
        asyncio.create_task(
            broadcast_telemetry_loop(ble, app.state.clients),
            name="telem_broadcast",
        ),
        asyncio.create_task(
            deadman_timer(ble, last_cmd_time),
            name="deadman",
        ),
    ]

    # --- graceful shutdown on SIGINT / SIGTERM ---------------------------
    shutdown_event = asyncio.Event()

    def _signal_handler() -> None:
        logger.info("Shutdown signal received")
        shutdown_event.set()

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, _signal_handler)
        except NotImplementedError:
            # Windows doesn't support add_signal_handler for SIGTERM;
            # SIGINT (Ctrl-C) will raise KeyboardInterrupt instead.
            pass

    try:
        # Wait until any task finishes (crash) or shutdown is requested.
        done, pending = await asyncio.wait(
            tasks,
            return_when=asyncio.FIRST_COMPLETED,
        )
        for t in done:
            if t.exception():
                logger.error("Task %s crashed: %s", t.get_name(), t.exception())
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt — shutting down")
    finally:
        # Cancel all remaining tasks
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        logger.info("All tasks stopped.  Goodbye.")


def main() -> None:
    try:
        asyncio.run(_run())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
