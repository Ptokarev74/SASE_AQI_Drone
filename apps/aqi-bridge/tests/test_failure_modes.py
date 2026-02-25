"""
Explicit tests validating Failure Mode Coverage and Invariant Enforcements.

These tests prove the mathematical and timing boundaries explicitly defined
for command staleness dropouts, pathological payload bounds, and crash-loop
mitigation logic.
"""

import asyncio
import sys
import time
from unittest.mock import Mock, patch

import pytest
from pydantic import ValidationError

from aqi_bridge.app import MAX_COMMAND_AGE_MS, main
from aqi_bridge.ble import BLEDroneClient
from aqi_bridge.models import ControlCommand

@pytest.fixture
def mock_ble():
    ble = BLEDroneClient()
    ble.connected.set()
    return ble


def test_pathological_payload_bounds():
    """
    Assert that the struct validation hard-rejects pathological mathematically-valid 
    JSON (NaN, Infinity, excessive out-of-range floats) BEFORE it hits logic processing.
    """
    # Test Out of Range
    with pytest.raises(ValidationError) as exc_info:
        ControlCommand(vx=99999.0, vy=0.0, vz=0.0, yaw=0.0)
    assert "Input should be less than or equal to 10000" in str(exc_info.value)
    
    # Test NaN rejection (using pydantic's allow_inf_nan=False)
    with pytest.raises(ValidationError):
        ControlCommand.model_validate({"vx": float("nan"), "vy": 0, "vz": 0, "yaw": 0})
        
    with pytest.raises(ValidationError):
        ControlCommand.model_validate({"vx": float("inf"), "vy": 0, "vz": 0, "yaw": 0})


@pytest.mark.asyncio
async def test_staleness_drop_enforcement(mock_ble, monkeypatch):
    """
    Assert that the BLE command consumer loop silently refuses to write commands 
    that have aged beyond the `MAX_COMMAND_AGE_MS` budget.
    """
    queue: asyncio.Queue[ControlCommand] = asyncio.Queue()
    
    # Create command and artificially age its receipt timestamp
    stale_cmd = ControlCommand(vx=50.0, arm=True)
    stale_cmd.ts_received = time.monotonic() - (MAX_COMMAND_AGE_MS / 1000.0) - 0.05
    
    await queue.put(stale_cmd)
    
    # Mock `ble._write_command_bytes` to track invocation
    write_mock = Mock(return_value=True)
    monkeypatch.setattr(mock_ble, "_write_command_bytes", write_mock)
    
    from aqi_bridge.app import command_consumer_loop
    task = asyncio.create_task(
        command_consumer_loop(mock_ble, queue, [time.monotonic()], {
            "dropped": 0, 
            "dropped_stale": 0,
            "errors": 0,
            "max_command_age_ms": 0.0,
            "sum_command_age_ms": 0.0,
            "total_commands_checked": 0
        })
    )
    
    # Let the queue loop spin once
    await asyncio.sleep(0.01)
    
    # The queue should be empty (it was popped)
    assert queue.empty()
    # BUT the write function MUST NOT have been called due to staleness
    write_mock.assert_not_called()
    
    task.cancel()


def test_supervisor_exponential_backoff():
    """
    Assert that the main runtime wrapper exponentially backs off when _run crashes,
    capping exactly at 60.0 seconds to prevent total starvation or hot-looping.
    """
    crashes_simulated = 0
    max_crashes = 8
    sleep_durations = []

    async def mock_run():
        nonlocal crashes_simulated
        crashes_simulated += 1
        if crashes_simulated >= max_crashes:
            raise KeyboardInterrupt() # Induce exit cleanly
        raise Exception("Simulated fatal bridge crash")
        
    def mock_sleep(seconds):
        sleep_durations.append(seconds)
        
    with patch("aqi_bridge.app._run", new=mock_run):
        with patch("time.sleep", side_effect=mock_sleep):
            # Should ultimately exit due to the KeyboardInterrupt induced at crash 8
            with pytest.raises(SystemExit) as exc:
                main()
            assert exc.value.code == 0
            
    # Expected sequence: 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 60.0, 60.0
    assert len(sleep_durations) == 7
    assert sleep_durations == [1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 60.0]
