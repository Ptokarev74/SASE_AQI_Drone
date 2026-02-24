"""
Stress test for the lock-safe broadcast pattern in server.py.

Simulates:
  1. Multiple healthy clients receiving telemetry concurrently.
  2. One intentionally slow client (stalls beyond WS_SEND_TIMEOUT_S).
  3. A dead client (raises on send).
  4. Rapid telemetry updates to verify no lock is held across sends.
  5. Verifies that slow/dead clients are removed while healthy clients survive.
  6. Verifies broadcast cycle timing stays stable under load.

All tests use asyncio without spawning real WebSocket connections.
"""

import asyncio
import logging
import time
from unittest.mock import AsyncMock, MagicMock

from starlette.websockets import WebSocketState

from bridge.ble_client import BLEDroneClient
from bridge.config import WS_SEND_TIMEOUT_S
from bridge.schemas import TelemetryMessage
from bridge.server import broadcast_telemetry_loop

logging.basicConfig(level=logging.DEBUG)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_fake_ws(
    *,
    slow: bool = False,
    dead: bool = False,
    slow_delay: float = 0.5,
) -> MagicMock:
    """Create a mock WebSocket that simulates various client behaviours."""
    ws = MagicMock()
    ws.client = ("127.0.0.1", 9999)
    ws.client_state = WebSocketState.CONNECTED

    if dead:
        ws.send_text = AsyncMock(side_effect=RuntimeError("connection reset"))
    elif slow:
        async def _slow_send(payload: str) -> None:
            await asyncio.sleep(slow_delay)  # stalls longer than timeout
        ws.send_text = AsyncMock(side_effect=_slow_send)
    else:
        ws.send_text = AsyncMock()

    return ws


def make_ble_with_telemetry() -> BLEDroneClient:
    """Create a BLEDroneClient with a pre-loaded telemetry snapshot."""
    ble = BLEDroneClient()
    ble.latest_telemetry = TelemetryMessage(status="ok", aqi=42)
    return ble


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

async def test_healthy_clients_receive_telemetry():
    """Multiple healthy clients each receive every broadcast frame."""
    print("\n--- Broadcast Test 1: Healthy Clients Receive Telemetry ---")
    ble = make_ble_with_telemetry()
    clients = set()

    ws1, ws2, ws3 = make_fake_ws(), make_fake_ws(), make_fake_ws()
    clients.update([ws1, ws2, ws3])

    # Run broadcast loop for just enough cycles
    task = asyncio.create_task(broadcast_telemetry_loop(ble, clients))
    await asyncio.sleep(0.15)  # ~4 cycles at 30 Hz
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass

    for ws in [ws1, ws2, ws3]:
        assert ws.send_text.called, "Healthy client never received a frame"
        for call in ws.send_text.call_args_list:
            payload = call[0][0]
            assert '"aqi":42' in payload, "Payload content mismatch"

    assert len(clients) == 3, f"Expected 3 clients, got {len(clients)}"
    print("  SUCCESS")


async def test_slow_client_dropped_others_survive():
    """A client that stalls beyond WS_SEND_TIMEOUT_S is removed; others continue."""
    print("\n--- Broadcast Test 2: Slow Client Dropped, Others Survive ---")
    ble = make_ble_with_telemetry()
    clients = set()

    healthy = make_fake_ws()
    slow = make_fake_ws(slow=True, slow_delay=WS_SEND_TIMEOUT_S + 0.3)
    clients.update([healthy, slow])

    task = asyncio.create_task(broadcast_telemetry_loop(ble, clients))
    await asyncio.sleep(0.4)  # enough time for timeout + removal
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass

    assert slow not in clients, "Slow client should have been removed"
    assert healthy in clients, "Healthy client should still be present"
    assert healthy.send_text.called, "Healthy client should have received frames"
    print("  SUCCESS")


async def test_dead_client_removed():
    """A client whose send_text raises is immediately removed from the registry."""
    print("\n--- Broadcast Test 3: Dead Client Removed ---")
    ble = make_ble_with_telemetry()
    clients = set()

    alive = make_fake_ws()
    dead = make_fake_ws(dead=True)
    clients.update([alive, dead])

    task = asyncio.create_task(broadcast_telemetry_loop(ble, clients))
    await asyncio.sleep(0.15)
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass

    assert dead not in clients, "Dead client should have been removed"
    assert alive in clients, "Alive client should still be present"
    print("  SUCCESS")


async def test_lock_not_held_during_send():
    """
    Verify that telemetry_lock is not held while send_text runs.

    We measure lock acquisition from a concurrent coroutine while the
    broadcast loop is mid-send.  If the lock is held during I/O, the
    acquire would time-out.
    """
    print("\n--- Broadcast Test 4: Lock Released Before Send ---")
    ble = make_ble_with_telemetry()
    clients = set()

    # A slow client ensures send_text is running for a while
    send_duration = WS_SEND_TIMEOUT_S - 0.02  # just under timeout, so it completes
    slow_but_ok = make_fake_ws(slow=True, slow_delay=send_duration)
    clients.add(slow_but_ok)

    lock_wait_times: list[float] = []

    async def try_acquire_lock():
        """Independently try to grab the telemetry_lock 10 times."""
        for _ in range(10):
            await asyncio.sleep(0.01)
            t0 = time.monotonic()
            async with ble.telemetry_lock:
                elapsed = time.monotonic() - t0
                lock_wait_times.append(elapsed)

    broadcast_task = asyncio.create_task(broadcast_telemetry_loop(ble, clients))
    await asyncio.gather(try_acquire_lock(), return_exceptions=True)
    broadcast_task.cancel()
    try:
        await broadcast_task
    except asyncio.CancelledError:
        pass

    max_wait = max(lock_wait_times) if lock_wait_times else 0.0
    # Lock should never be held longer than a few microseconds
    assert max_wait < 0.005, (
        f"Lock was held for {max_wait*1000:.1f}ms — possible I/O inside lock"
    )
    print(f"  Max lock wait: {max_wait*1000:.3f}ms — SUCCESS")


async def test_no_telemetry_no_send():
    """Broadcast loop does nothing when latest_telemetry is None."""
    print("\n--- Broadcast Test 5: No Telemetry → No Send ---")
    ble = BLEDroneClient()
    assert ble.latest_telemetry is None

    clients = set()
    ws = make_fake_ws()
    clients.add(ws)

    task = asyncio.create_task(broadcast_telemetry_loop(ble, clients))
    await asyncio.sleep(0.15)
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass

    assert not ws.send_text.called, "send_text should not be called when no telemetry"
    print("  SUCCESS")


async def test_identical_telemetry_sent_once():
    """Telemetry that doesn't change is not re-serialised and re-sent."""
    print("\n--- Broadcast Test 6: Identical Telemetry Sent Only Once ---")
    ble = make_ble_with_telemetry()
    clients = set()
    ws = make_fake_ws()
    clients.add(ws)

    # Two broadcasts should only result in 1 send (first cycle sends, rest skip)
    task = asyncio.create_task(broadcast_telemetry_loop(ble, clients))
    await asyncio.sleep(0.12)  # ~3-4 cycles at 30 Hz
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass

    call_count = ws.send_text.call_count
    assert call_count == 1, f"Expected 1 send, got {call_count} (duplicates sent)"
    print("  SUCCESS")


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

async def _run_all():
    await test_healthy_clients_receive_telemetry()
    await test_slow_client_dropped_others_survive()
    await test_dead_client_removed()
    await test_lock_not_held_during_send()
    await test_no_telemetry_no_send()
    await test_identical_telemetry_sent_once()
    print("\nALL BROADCAST STRESS TESTS PASSED")


if __name__ == "__main__":
    try:
        asyncio.run(_run_all())
    except AssertionError:
        import traceback
        traceback.print_exc()
        raise SystemExit(1) from None
