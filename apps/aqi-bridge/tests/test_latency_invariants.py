"""
Tests functional constraints and invariant bounds for command pipeline latency.

NOTE ON CI ENVIRONMENT:
These tests validate that the pipeline strictly captures monotonic timestamps
and correctly sequences events (ts_received <= ts_enqueued <= ts_dequeued).
It enforces a very loose MAXIMUM boundary (500ms) to detect absolute starvation
within shared CI runners.

These tests DO NOT assert hard real-time determinism. Real-time qualification
must be run via `tests/test_perf_qualify.py` on dedicated, deterministic hardware
with strictly enforced 120ms budgets.
"""

import asyncio
from unittest.mock import AsyncMock

import pytest

from aqi_bridge.app import command_consumer_loop
from aqi_bridge.api import _handle_control_message
from aqi_bridge.ble import BLEDroneClient
from aqi_bridge.models import ControlCommand

CI_FUNCTIONAL_LENGHTY_LATENCY_MAX_S = 0.5  # 500ms functional cap for noisy CI


@pytest.mark.asyncio
async def test_latency_timestamps_strictly_ordered():
    """Prove pipeline captures monotonic clock events in strictly ascending order."""
    ble = AsyncMock(spec=BLEDroneClient)
    ble.connected = asyncio.Event()
    ble.connected.set()
    ble._write_command_bytes = AsyncMock(return_value=True)

    queue = asyncio.Queue(maxsize=10)
    last_cmd_time = [0.0]
    write_metrics = {
        "success": 0,
        "errors": 0, 
        "dropped": 0, 
        "dropped_stale": 0,
        "last_error_timestamp": 0.0,
        "max_command_age_ms": 0.0,
        "sum_command_age_ms": 0.0,
        "total_commands_checked": 0
    }

    # 1. Inject JSON (simulating WebSocket receipt)
    raw_json = '{"vx": 1.0, "yaw": 0.5, "arm": true}'
    
    # Send message which enqueues it with ts_received and ts_enqueued
    _handle_control_message(raw_json, queue)
    
    # 2. Run the consumer loop for a single iteration
    consumer_task = asyncio.create_task(
        command_consumer_loop(ble, queue, last_cmd_time, write_metrics)
    )
    
    # Wait for the mock write to be called
    await asyncio.sleep(0.01)
    consumer_task.cancel()
    
    try:
        await consumer_task
    except asyncio.CancelledError:
        pass

    assert ble._write_command_bytes.call_count == 1
    
    # 3. Retrieve the dispatched command
    dispatched_cmd: ControlCommand = ble._write_command_bytes.call_args[0][0]

    # Invariants: 
    # Must be > 0 (actually populated)
    assert dispatched_cmd.ts_received > 0.0
    
    # Logical timeline constraints
    assert dispatched_cmd.ts_received <= dispatched_cmd.ts_enqueued
    assert dispatched_cmd.ts_enqueued <= dispatched_cmd.ts_dequeued
    assert dispatched_cmd.ts_dequeued <= dispatched_cmd.ts_dispatched

    # Compute latencies
    queue_dwell_time = dispatched_cmd.ts_dequeued - dispatched_cmd.ts_enqueued
    end_to_end_latency = dispatched_cmd.ts_dispatched - dispatched_cmd.ts_received

    # Enforce CI-bound functional starvation assertions
    assert queue_dwell_time < CI_FUNCTIONAL_LENGHTY_LATENCY_MAX_S, "Queue dwell exceeded CI bounds"
    assert end_to_end_latency < CI_FUNCTIONAL_LENGHTY_LATENCY_MAX_S, "E2E latency exceeded CI bounds"

