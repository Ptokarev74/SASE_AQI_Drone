"""
Explicit tests validating dual-layer Command Freshness Enforcement.

These tests assert that standard drop-oldest buffering is insufficient alone,
and explicit timing constraints categorically reject stale payload intents both:
1. Immediately upon dequeue
2. Immediately before the BLE transmit system call.
"""

import asyncio
import time
from unittest.mock import AsyncMock

import pytest
from aqi_bridge.app import command_consumer_loop
from aqi_bridge.ble import BLEDroneClient
from aqi_bridge.config import MAX_COMMAND_AGE_MS
from aqi_bridge.models import ControlCommand


@pytest.fixture
def mock_ble():
    ble = BLEDroneClient()
    ble.connected.set()
    return ble

@pytest.fixture
def base_metrics():
    return {
        "success": 0,
        "dropped": 0,
        "dropped_stale": 0,
        "errors": 0,
        "last_error_timestamp": 0.0,
        "max_command_age_ms": 0.0,
        "sum_command_age_ms": 0.0,
        "total_commands_checked": 0,
    }


@pytest.mark.asyncio
async def test_freshness_dequeue_boundary(mock_ble, base_metrics, monkeypatch):
    """
    Assert commands exceeding the exact threshold boundary at the moment 
    of dequeue are dropped and tracked correctly.
    """
    queue: asyncio.Queue[ControlCommand] = asyncio.Queue()
    
    # SAFELY WITHIN THRESHOLD: 290.0ms (Should pass)
    cmd_pass = ControlCommand(vx=10.0)
    cmd_pass.ts_received = time.monotonic() - ((MAX_COMMAND_AGE_MS - 10.0) / 1000.0)
    await queue.put(cmd_pass)
    
    # JUST PAST THRESHOLD: 310.0ms (Should drop)
    cmd_drop = ControlCommand(vx=20.0)
    cmd_drop.ts_received = time.monotonic() - ((MAX_COMMAND_AGE_MS + 10.0) / 1000.0)
    await queue.put(cmd_drop)
    
    write_mock = AsyncMock(return_value=True)
    monkeypatch.setattr(mock_ble, "_write_command_bytes", write_mock)
    
    try:
        task = asyncio.create_task(command_consumer_loop(mock_ble, queue, [time.monotonic()], base_metrics))
        
        # Spin the loop enough to process both commands
        await asyncio.sleep(0.05)
        
        # The queue should be empty
        assert queue.empty()
        
        # Only ONE write should have occurred (the exactly-on-threshold one)
        write_mock.assert_called_once()
        assert write_mock.call_args[0][0].vx == 10.0
        
        # Metrics assertion
        assert base_metrics["dropped_stale"] == 1
        assert base_metrics["total_commands_checked"] == 2
        
    finally:
        task.cancel()
        import contextlib
        with contextlib.suppress(asyncio.CancelledError):
            await task


@pytest.mark.asyncio
async def test_freshness_pre_write_artificial_stall(mock_ble, base_metrics, monkeypatch):
    """
    Simulate an execution thread stall occurring *between* the dequeue check
    and the actual BLE network call. 
    Assert that the PRE-WRITE layer catches the stale command.
    """
    queue: asyncio.Queue[ControlCommand] = asyncio.Queue()
    
    # Create a fresh command (0ms age initially)
    cmd = ControlCommand(vx=50.0)
    cmd.ts_received = time.monotonic()
    await queue.put(cmd)
    
    write_mock = AsyncMock(return_value=True)
    
    # We patch the ble client BUT we simulate the write itself taking
    # an exceptionally long time (blocking the loop artificially).
    # To trigger the pre-write check, we'll actually use `patch` to intercept
    # the function and stall *before* it returns... wait, the check is IN the bridge logic before the call.
    # To test the bridge logic itself, we must artificially stall the bridge loop before it hits the layer.
    
    # The simplest way to test the pre-write check is to hijack `ble.connected.is_set()` 
    # to sleep and stall the thread right before the pre-write check is executed.
    
    original_is_set = mock_ble.connected.is_set
    
    def stalling_is_set():
        # Artificial stall for longer than MAX_COMMAND_AGE_MS
        time.sleep((MAX_COMMAND_AGE_MS / 1000.0) + 0.1)
        return original_is_set()
        
    mock_ble.connected.is_set = stalling_is_set
    monkeypatch.setattr(mock_ble, "_write_command_bytes", write_mock)
    
    try:
        task = asyncio.create_task(command_consumer_loop(mock_ble, queue, [time.monotonic()], base_metrics))
        
        await asyncio.sleep(0.05) # Give context switch to the task
        
        # Queue emptied
        assert queue.empty()
        
        # The bridge logic hit the pre-write check AFTER the stall and should have aborted the write
        write_mock.assert_not_called()
        assert base_metrics["dropped_stale"] == 1
        
    finally:
        task.cancel()
        import contextlib
        with contextlib.suppress(asyncio.CancelledError):
            await task
