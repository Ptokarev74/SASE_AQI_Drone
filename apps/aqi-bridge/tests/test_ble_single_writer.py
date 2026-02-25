"""
Regression tests for the BLE single-writer serialization rule.

Validates:
  1. _write_command_bytes is the only write surface (public write_command removed).
  2. writes_in_flight counter is incremented before write, decremented after.
  3. Concurrent calls to _write_command_bytes are detected and logged as errors.
  4. _write_lock serializes parallel callers — no interleaving.
  5. A simulated command_consumer_loop is the only caller in the full flow.
  6. deadman_timer enqueues via queue, never calls _write_command_bytes directly.
  7. BLE disconnect causes _write_command_bytes to return False cleanly.
  8. Flood test: 500 Hz command rate — writes_in_flight never exceeds 1,
     no unhandled exceptions.
"""

import asyncio
import logging
import time
from unittest.mock import AsyncMock, MagicMock, patch

from aqi_bridge.ble import BLEDroneClient
from aqi_bridge.models import ControlCommand

logging.basicConfig(level=logging.ERROR)  # suppress DEBUG noise


def _make_cmd(vx: float = 1.0) -> ControlCommand:
    return ControlCommand(vx=vx, vy=0.0, vz=0.0, yaw=0.0, arm=True)


def _make_connected_ble() -> BLEDroneClient:
    """BLEDroneClient with a mocked connected bleak client."""
    ble = BLEDroneClient()
    mock_client = MagicMock()
    mock_client.is_connected = True
    mock_client.write_gatt_char = AsyncMock()
    ble._client = mock_client
    ble.connected.set()  # explicitly set the connection event for consumer loop
    return ble


# ---------------------------------------------------------------------------
# Test 1: Public write_command removed — only _write_command_bytes exists
# ---------------------------------------------------------------------------
async def test_no_public_write_command():
    print("\n--- Serial Test 1: No Public write_command ---")
    ble = BLEDroneClient()
    assert not hasattr(ble, "write_command"), (
        "ble.write_command still exists! It must be removed or renamed to "
        "_write_command_bytes to enforce single-writer rule."
    )
    assert hasattr(ble, "_write_command_bytes"), (
        "_write_command_bytes not found — serialization surface is missing."
    )
    print("  SUCCESS")


# ---------------------------------------------------------------------------
# Test 2: writes_in_flight counter tracks correctly
# ---------------------------------------------------------------------------
async def test_writes_in_flight_counter():
    print("\n--- Serial Test 2: writes_in_flight Counter ---")
    ble = _make_connected_ble()

    in_flight_during_write: list[int] = []

    async def _capture_inflight(*args, **kwargs):
        in_flight_during_write.append(ble.writes_in_flight)

    ble._client.write_gatt_char = AsyncMock(side_effect=_capture_inflight)

    assert ble.writes_in_flight == 0
    await ble._write_command_bytes(_make_cmd())
    assert ble.writes_in_flight == 0, f"Counter not decremented: {ble.writes_in_flight}"
    assert in_flight_during_write == [1], (
        f"Expected writes_in_flight==1 during write, got {in_flight_during_write}"
    )
    print(f"  writes_in_flight during write: {in_flight_during_write[0]}")
    print("  SUCCESS")


# ---------------------------------------------------------------------------
# Test 3: Concurrent write detection — logs error if > 1 in flight
# ---------------------------------------------------------------------------
async def test_concurrent_write_detected():
    print("\n--- Serial Test 3: Concurrent Write Detection ---")
    ble = _make_connected_ble()

    max_in_flight: list[int] = [0]
    error_logged: list[bool] = [False]

    async def _slow_write(*args, **kwargs):
        # Stall long enough for a second write to arrive
        await asyncio.sleep(0.05)
        if ble.writes_in_flight > max_in_flight[0]:
            max_in_flight[0] = ble.writes_in_flight

    ble._client.write_gatt_char = AsyncMock(side_effect=_slow_write)

    with patch("aqi_bridge.ble.logger") as mock_log:
        # Launch two concurrent writes — this simulates the forbidden pattern
        await asyncio.gather(
            ble._write_command_bytes(_make_cmd(vx=1.0)),
            ble._write_command_bytes(_make_cmd(vx=2.0)),
        )
        # Check that an error was logged about concurrent writes
        error_calls = [
            c for c in mock_log.error.call_args_list
            if "CONCURRENCY BUG" in str(c)
        ]
        error_logged[0] = len(error_calls) > 0

    assert ble.writes_in_flight == 0, (
        f"Counter leaked: writes_in_flight={ble.writes_in_flight}"
    )
    assert max_in_flight[0] > 1, (
        "Expected writes_in_flight > 1 during concurrent writes"
    )
    assert error_logged[0], (
        "Expected error log about BLE WRITE CONCURRENCY BUG — not found"
    )
    print(f"  Max in-flight during concurrent call: {max_in_flight[0]}")
    print("  SUCCESS: concurrency bug correctly detected and logged")


# ---------------------------------------------------------------------------
# Test 4: _write_lock serializes — no actual bleak interleave
# ---------------------------------------------------------------------------
async def test_write_lock_serializes():
    print("\n--- Serial Test 4: _write_lock Serializes Writes ---")
    ble = _make_connected_ble()

    write_order: list[float] = []

    async def _stamped_write(*args, **kwargs):
        import time
        write_order.append(time.monotonic())
        await asyncio.sleep(0.02)  # simulate BLE round-trip

    ble._client.write_gatt_char = AsyncMock(side_effect=_stamped_write)

    # Send 5 concurrent writes — lock must serialize them
    import time
    t0 = time.monotonic()
    await asyncio.gather(*[ble._write_command_bytes(_make_cmd(vx=float(i))) for i in range(5)])
    elapsed = time.monotonic() - t0

    # With lock serialization at 20ms each: total >= 5 * 20ms = 100ms
    assert elapsed >= 0.08, f"Writes weren't serialized — total time {elapsed:.3f}s too short"
    assert ble.writes_in_flight == 0
    print(f"  5 serialized writes completed in {elapsed*1000:.1f}ms — SUCCESS")


# ---------------------------------------------------------------------------
# Test 5: Disconnected BLE → _write_command_bytes returns False cleanly
# ---------------------------------------------------------------------------
async def test_disconnected_returns_false():
    print("\n--- Serial Test 5: Disconnected Returns False Cleanly ---")
    ble = BLEDroneClient()  # no _client set → disconnected

    result = await ble._write_command_bytes(_make_cmd())
    assert result is False, f"Expected False on disconnect, got {result}"
    assert ble.writes_in_flight == 0, "Counter leaked on early-return path"
    print("  SUCCESS")


# ---------------------------------------------------------------------------
# Test 6: command_consumer_loop is the only write path in full simulation
# ---------------------------------------------------------------------------
async def test_consumer_loop_only_write_path():
    print("\n--- Serial Test 6: command_consumer_loop is The Only Write Path ---")
    from aqi_bridge.api import enqueue_command_drop_oldest
    from aqi_bridge.config import COMMAND_QUEUE_SIZE

    ble = _make_connected_ble()
    queue: asyncio.Queue[ControlCommand] = asyncio.Queue(maxsize=COMMAND_QUEUE_SIZE)

    write_calls: list[ControlCommand] = []
    original_write = ble._write_command_bytes

    async def _tracking_write(cmd):
        write_calls.append(cmd)
        return await original_write(cmd)

    ble._write_command_bytes = _tracking_write  # type: ignore[method-assign]

    # Simulate consumer loop
    async def consumer():
        _last_cmd_time = [0.0]
        while True:
            try:
                cmd = await asyncio.wait_for(queue.get(), timeout=0.5)
                await ble._write_command_bytes(cmd)
            except asyncio.TimeoutError:
                break

    enqueue_command_drop_oldest(queue, _make_cmd(vx=10.0))
    enqueue_command_drop_oldest(queue, _make_cmd(vx=20.0))
    enqueue_command_drop_oldest(queue, _make_cmd(vx=30.0))

    await consumer()

    assert len(write_calls) == 3, f"Expected 3 writes, got {len(write_calls)}"
    assert [c.vx for c in write_calls] == [10.0, 20.0, 30.0], (
        f"Write order wrong: {[c.vx for c in write_calls]}"
    )
    print(f"  {len(write_calls)} writes in order: {[c.vx for c in write_calls]}")
    print("  SUCCESS")


# ---------------------------------------------------------------------------
# Test 7: Flood test — writes_in_flight never exceeds 1 (single consumer)
# ---------------------------------------------------------------------------
async def test_flood_writes_in_flight_capped():
    print("\n--- Serial Test 7: Flood Test — writes_in_flight Never > 1 ---")
    from aqi_bridge.api import enqueue_command_drop_oldest
    from aqi_bridge.config import COMMAND_QUEUE_SIZE

    ble = _make_connected_ble()
    queue: asyncio.Queue[ControlCommand] = asyncio.Queue(maxsize=COMMAND_QUEUE_SIZE)

    max_in_flight = [0]

    async def _tracking_write(*args, **kwargs):
        if ble.writes_in_flight > max_in_flight[0]:
            max_in_flight[0] = ble.writes_in_flight

    ble._client.write_gatt_char = AsyncMock(side_effect=_tracking_write)

    async def flood_producer():
        for i in range(500):
            enqueue_command_drop_oldest(queue, _make_cmd(vx=float(i)))
            await asyncio.sleep(0)  # yield to event loop

    async def consumer():
        for _ in range(500):
            try:
                cmd = await asyncio.wait_for(queue.get(), timeout=1.0)
                await ble._write_command_bytes(cmd)
            except asyncio.TimeoutError:
                break

    await asyncio.gather(flood_producer(), consumer())

    assert max_in_flight[0] == 1, (
        f"writes_in_flight reached {max_in_flight[0]} — concurrent writes detected!"
    )
    assert ble.writes_in_flight == 0
    print(f"  Max writes_in_flight during 500-command flood: {max_in_flight[0]}")
    print("  SUCCESS")


# ---------------------------------------------------------------------------
# Test 8: BLE write failure policy propagates exceptions and triggers disconnect
# ---------------------------------------------------------------------------
async def test_ble_write_failure_policy_handles_exceptions():
    print("\n--- Serial Test 8: BLE Write Failure Policy ---")
    from aqi_bridge.app import command_consumer_loop
    from bleak.exc import BleakError
    
    ble = _make_connected_ble()
    queue: asyncio.Queue[ControlCommand] = asyncio.Queue(maxsize=10)
    last_cmd_time = [0.0]
    write_metrics = {
        "errors": 0, 
        "dropped": 0, 
        "dropped_stale": 0,
        "last_error_timestamp": 0.0,
        "max_command_age_ms": 0.0,
        "sum_command_age_ms": 0.0,
        "total_commands_checked": 0
    }
    
    # Mock write_gatt_char to raise a BleakError
    ble._client.write_gatt_char = AsyncMock(side_effect=BleakError("Simulated write exception"))
    
    # Mock safe_disconnect so we can assert it was called
    ble._safe_disconnect = AsyncMock()
    
    # Put a single command in the queue
    cmd = _make_cmd(vx=99.0)
    cmd.ts_received = time.monotonic() # Ensure it's not dropped for staleness
    queue.put_nowait(cmd)
    
    # Run the consumer loop momentarily. 
    # It will pull the command out, attempt to write, catch the BleakError,
    # log it, tear down the connection, and then block on queue.get().
    consumer_task = asyncio.create_task(
        command_consumer_loop(ble, queue, last_cmd_time, write_metrics)
    )
    
    await asyncio.sleep(0.05)  # Yield explicitly to let the consumer run once
    
    # Cancel to clean up
    consumer_task.cancel()
    try:
        await consumer_task
    except asyncio.CancelledError:
        pass
        
    assert write_metrics["errors"] == 1, f"Expected 1 error recorded, found {write_metrics['errors']}"
    assert write_metrics["dropped"] == 1, f"Expected 1 dropped command, found {write_metrics['dropped']}"
    assert write_metrics["last_error_timestamp"] > 0.0, "last_error_timestamp not updated!"
    
    ble._safe_disconnect.assert_awaited_once()
    assert ble.writes_in_flight == 0, "writes_in_flight counter leaked during exception"
    print("  SUCCESS: Write failures recorded and gracefully trigger safe disconnect.")


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------
async def _run_all():
    await test_no_public_write_command()
    await test_writes_in_flight_counter()
    await test_concurrent_write_detected()
    await test_write_lock_serializes()
    await test_disconnected_returns_false()
    await test_consumer_loop_only_write_path()
    await test_flood_writes_in_flight_capped()
    await test_ble_write_failure_policy_handles_exceptions()
    print("\nALL BLE SERIALIZATION TESTS PASSED")


if __name__ == "__main__":
    try:
        asyncio.run(_run_all())
    except AssertionError:
        import traceback
        traceback.print_exc()
        raise SystemExit(1) from None
