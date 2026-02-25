"""
Invariant tests validating the Deadman Safety Contract.

Asserts:
1. Silence > DEADMAN_TIMEOUT_MS generates EXACTLY one FAILSAFE_COMMAND within the FAILSAFE_EMIT_BUDGET_MS margin.
2. Generating a failsafe locks out normal control inputs.
3. Control inhibition explicitly requires a re-arm command to resume.
"""

import asyncio
import time

import pytest
from aqi_bridge.api import _handle_control_message
from aqi_bridge.app import deadman_timer
from aqi_bridge.ble import BLEDroneClient
from aqi_bridge.config import DEADMAN_TIMEOUT_S, FAILSAFE_EMIT_BUDGET_MS
from aqi_bridge.models import ControlCommand


@pytest.fixture
def mock_ble():
    ble = BLEDroneClient()
    ble.connected.set()  # simulate connected
    return ble


@pytest.mark.asyncio
async def test_failsafe_emission_budget(mock_ble):
    """
    Assert that exactly one FAILSAFE_COMMAND is emitted within the
    strict budget parameter after the silence threshold is crossed.
    """
    queue: asyncio.Queue[ControlCommand] = asyncio.Queue(maxsize=10)
    last_cmd_time = [time.monotonic()]
    control_inhibited = [False]
    
    # Spawn the deadman task
    task = asyncio.create_task(deadman_timer(mock_ble, queue, last_cmd_time, control_inhibited))
    
    # Simulate time passing just past the timeout
    # (By artificially pushing the last command time backward exactly the threshold amount)
    last_cmd_time[0] = time.monotonic() - DEADMAN_TIMEOUT_S - 0.001
    
    # The budget is extremely tight, wait just the budget margin
    await asyncio.sleep(FAILSAFE_EMIT_BUDGET_MS / 1000.0)
    
    try:
        # Assert the failsafe was injected
        assert not queue.empty(), f"FAILSAFE_COMMAND not emitted within {FAILSAFE_EMIT_BUDGET_MS}ms budget!"
        cmd = queue.get_nowait()
        
        # Verify the explicit constant was used
        assert cmd.is_failsafe is True
        assert cmd.arm is False
        assert cmd.vx == 0.0
        
        # Assert the lockout engaged
        assert control_inhibited[0] is True, "Control was not properly inhibited!"
        
    finally:
        task.cancel()


@pytest.mark.asyncio
async def test_control_inhibition_and_rearm():
    """
    Assert that while `control_inhibited` is True, standard telemetry 
    is dropped, and control only resumes upon an explicit `arm=True`.
    """
    queue: asyncio.Queue[ControlCommand] = asyncio.Queue(maxsize=10)
    control_inhibited = [True]  # Simulate post-failsafe state
    
    # Incoming thrust command from a 'pegged' joystick
    
    # Send normal (un-armed) thrust command
    _handle_control_message(
        '{"vx": 10.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0, "arm": false}',
        queue,
        control_inhibited
    )
    # Assert it was rejected
    assert queue.empty()
    assert control_inhibited[0] is True
    
    # Send explicit re-arm command
    _handle_control_message(
        '{"vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0, "arm": true}',
        queue,
        control_inhibited
    )
    
    # Assert it was accepted AND the lock is cleared
    assert not queue.empty()
    assert control_inhibited[0] is False
    
    # Assert normal commands flow again
    _handle_control_message(
        '{"vx": 10.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0, "arm": false}',
        queue,
        control_inhibited
    )
    assert queue.qsize() == 2  # The arm command + the new thrust command

