"""
Orchestrator — ties BLE client, WebSocket server, deadman timer, and
command consumer together under a single asyncio event loop.

Run with:
    python -m bridge.main

BLE Write Serialization Rule
-----------------------------
All BLE writes flow through a single coroutine: ``command_consumer_loop``.

  ALLOWED:   command_consumer_loop  →  ble._write_command_bytes()
  FORBIDDEN: deadman_timer          →  ble._write_command_bytes()  ← BUG
  FORBIDDEN: websocket_endpoint     →  ble._write_command_bytes()  ← BUG
  FORBIDDEN: any other coroutine    →  ble._write_command_bytes()  ← BUG

The deadman timer and any other safety mechanism that needs to send a
command must enqueue it into `command_queue`.  The consumer loop is the
ONLY path to the BLE hardware.
"""

from __future__ import annotations

import asyncio
import logging
import signal
import sys
import time

import uvicorn

from bridge.ble_client import BLEDroneClient

# ---------------------------------------------------------------------------
# Logging Setup
# ---------------------------------------------------------------------------
from bridge.config import (
    COMMAND_QUEUE_SIZE,
    DEADMAN_TIMEOUT_S,
    LOG_FORMAT,
    LOG_LEVEL,
    LOG_RATE_LIMIT_S,
    LOOP_STARVATION_THRESHOLD_S,
    WS_HOST,
    WS_PORT,
)
from bridge.schemas import ZERO_COMMAND, ControlCommand
from bridge.server import broadcast_telemetry_loop, create_app, enqueue_command_drop_oldest


def setup_logging():
    """Configure structured logging based on config.LOG_FORMAT."""
    root_logger = logging.getLogger()
    root_logger.setLevel(LOG_LEVEL)

    # Clean up existing handlers
    for handler in root_logger.handlers[:]:
        root_logger.removeHandler(handler)

    handler = logging.StreamHandler()
    
    if LOG_FORMAT == "json":
        try:
            from pythonjsonlogger import jsonlogger
            formatter = jsonlogger.JsonFormatter(
                "%(asctime)s %(levelname)s %(name)s %(message)s"
            )
        except ImportError:
            # Fallback if dependency missing
            logger_internal = logging.getLogger(__name__)
            logger_internal.warning("python-json-logger not found, falling back to text format")
            formatter = logging.Formatter("%(asctime)s  %(levelname)-8s  %(name)s  %(message)s")
    else:
        formatter = logging.Formatter("%(asctime)s  %(levelname)-8s  %(name)s  %(message)s")

    handler.setFormatter(formatter)
    root_logger.addHandler(handler)

setup_logging()
logger = logging.getLogger("bridge")



# ---------------------------------------------------------------------------
# Command consumer — THE ONLY BLE write path
# ---------------------------------------------------------------------------
async def command_consumer_loop(
    ble: BLEDroneClient,
    queue: asyncio.Queue[ControlCommand],
    last_cmd_time: list[float],
    write_metrics: dict[str, float],
) -> None:
    """
    Drain the command queue and write each command to BLE.

    This is the SOLE coroutine permitted to call ``ble._write_command_bytes()``.
    No other code path may perform BLE writes.

    On write success, updates ``last_cmd_time[0]`` so the deadman timer knows
    when the last real command was forwarded to the drone.

    Disconnect behavior: if BLE is disconnected, ``_write_command_bytes``
    returns False immediately.  The command is silently dropped — the queue
    keeps draining so it stays bounded and never stalls the producer.

    If the write raises an exception, the exception is caught, recorded into
    ``write_metrics``, the command is dropped, and the BLE connection is 
    explicitly torn down (``_safe_disconnect()``) to force a clean reconnection cycle.
    """
    while True:
        try:
            cmd = await queue.get()

            #ONLY try writing if already connected to prevent spam
            if not ble.connected.is_set():
                 write_metrics["dropped"] += 1
                 logger.debug("BLE write skipped (disconnected) — command dropped: %s", cmd)
                 continue

            try:
                # ONLY write surface for BLE in the entire process.
                ok = await ble._write_command_bytes(cmd)
                if ok:
                    last_cmd_time[0] = time.monotonic()
                    logger.debug("BLE write OK: %s", cmd)
                else:
                    write_metrics["dropped"] += 1
                    logger.debug(
                        "BLE write skipped (explicitly disconnected) — command dropped: %s", cmd
                    )
            except Exception as exc:
                write_metrics["errors"] += 1
                write_metrics["dropped"] += 1
                write_metrics["last_error_timestamp"] = time.monotonic()
                logger.error("BLE write failed deterministically: %s", exc)
                # Fail-fast: tear down the link to force a clean restart cycle
                await ble._safe_disconnect()

        except asyncio.CancelledError:
            logger.info("command_consumer_loop cancelled")
            raise
        except Exception:
            logger.exception("Unexpected error in command_consumer_loop")


# ---------------------------------------------------------------------------
# Deadman safety — enqueues ZERO_COMMAND, does NOT write BLE directly
# ---------------------------------------------------------------------------
async def deadman_timer(
    ble: BLEDroneClient,
    queue: asyncio.Queue[ControlCommand],
    last_cmd_time: list[float],
) -> None:
    """
    If no control command has been received for ``DEADMAN_TIMEOUT_S`` seconds,
    push a zero-velocity / disarm command onto the command queue.

    IMPORTANT: This coroutine enqueues via ``enqueue_command_drop_oldest()``,
    NOT by calling ``ble._write_command_bytes()`` directly.  This preserves
    the single-writer invariant: all BLE writes happen in command_consumer_loop.

    This is critical safety logic: if the WebSocket client crashes, the phone
    loses connectivity, or the PWA tab is closed, the drone should stop.
    """
    fired = False  # avoid spamming zero commands

    while True:
        try:
            await asyncio.sleep(0.1)  # check at 10 Hz

            # Only enforce deadman when BLE is connected and we've received
            # at least one command (last_cmd_time != 0).
            if not ble.connected.is_set() or last_cmd_time[0] == 0.0:
                fired = False
                continue

            elapsed = time.monotonic() - last_cmd_time[0]

            if elapsed > DEADMAN_TIMEOUT_S:
                if not fired:
                    logger.warning(
                        "DEADMAN: no command for %.2fs — enqueuing ZERO_COMMAND",
                        elapsed,
                    )
                    # Enqueue via the same drop-oldest path — preserves single-writer rule.
                    enqueue_command_drop_oldest(queue, ZERO_COMMAND)
                    fired = True
            else:
                fired = False

        except asyncio.CancelledError:
            logger.info("deadman_timer cancelled")
            raise
        except Exception:
            logger.exception("Unexpected error in deadman_timer")


# ---------------------------------------------------------------------------
# Event Loop Watchdog
# ---------------------------------------------------------------------------
async def event_loop_watchdog(loop_lag: list[float], starvation_count: list[int]) -> None:
    """
    Actively monitors the asyncio event loop for starvation caused by blocking
    synchronous code or I/O floods.

    Measures drift between the expected wake time and actual wake time.
    If the drift exceeds `LOOP_STARVATION_THRESHOLD_S`, it logs a warning
    (rate-limited by `LOG_RATE_LIMIT_S`) and increments the starvation count
    accessible via the `/health` endpoint.
    """
    sleep_interval = 1.0
    last_log_time = 0.0

    while True:
        expected_wake = time.monotonic() + sleep_interval
        await asyncio.sleep(sleep_interval)
        actual_wake = time.monotonic()
        lag = actual_wake - expected_wake

        # Expose current lag for /health metric
        loop_lag[0] = lag

        if lag > LOOP_STARVATION_THRESHOLD_S:
            starvation_count[0] += 1
            if actual_wake - last_log_time >= LOG_RATE_LIMIT_S:
                logger.warning(
                    "EVENT LOOP STARVATION: watchdog woke up %.1fms late! "
                    "Total starvation events: %d. Check for blocking CPU operations.",
                    lag * 1000.0, starvation_count[0]
                )
                last_log_time = actual_wake


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
    
    # Watchdog metrics
    loop_lag: list[float] = [0.0]
    starvation_count: list[int] = [0]
    
    # BLE Write Failure metrics
    write_metrics = {
        "errors": 0,
        "dropped": 0,
        "last_error_timestamp": 0.0,
    }

    app = create_app(ble, command_queue, loop_lag, starvation_count, write_metrics)

    # --- spawn concurrent tasks -----------------------------------------
    tasks = [
        asyncio.create_task(ble.run_forever(), name="ble_loop"),
        asyncio.create_task(run_uvicorn(app), name="uvicorn"),
        asyncio.create_task(
            command_consumer_loop(ble, command_queue, last_cmd_time, write_metrics),
            name="cmd_consumer",
        ),
        asyncio.create_task(
            broadcast_telemetry_loop(ble, app.state.clients),
            name="telem_broadcast",
        ),
        asyncio.create_task(
            deadman_timer(ble, command_queue, last_cmd_time),
            name="deadman",
        ),
        asyncio.create_task(
            event_loop_watchdog(loop_lag, starvation_count),
            name="watchdog",
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
            exc = t.exception()
            if exc:
                logger.error("Task %s crashed: %s", t.get_name(), exc)
                # Fail-fast: Re-raise so asyncio.run() propagates it and we exit non-zero
                raise exc
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
        sys.exit(0)
    except Exception:
        # Task crash triggered an unhandled exception
        logger.critical("Bridge process exiting due to fatal error")
        sys.exit(1)


if __name__ == "__main__":
    main()
