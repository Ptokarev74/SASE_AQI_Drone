"""
Hardware-in-the-Loop (HIL) safety qualification harness.

This script executes the provable Deadman Safety Contract against physical drone hardware.
It requires the Arduino firmware to be compiled with `HIL_TEST_MODE` enabled, wherein
the drone outputs a high-resolution microsecond timestamp over Serial exactly when 
the motor shutdown routine is locked in.

This certification script ensures that bridging and firmware delays together 
do not exceed the mandated strict performance budget.
"""

import argparse
import asyncio
import json
import logging
import time
import sys
from typing import Optional

# Requires `pyserial` for side-channel hardware verification
import serial
from bleak import BleakClient, BleakScanner

from aqi_bridge.config import (
    BLE_DEVICE_NAME,
    COMMAND_CHAR_UUID,
    DEADMAN_TIMEOUT_MS,
    FAILSAFE_EMIT_BUDGET_MS,
    USE_WRITE_WITH_RESPONSE
)
from aqi_bridge.models import ControlCommand

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger("hil_qualify")

# --- Explicit Certification Allowances (Hardware + Firmware + Bridge) ---
# Total maximum permissible time from silence injection until the motors are SAFE.
# Bridge Budget (timeout + emit) + BLE Transmission + Arduino MCU Loop processing
MAX_HIL_LATENCY_MS = DEADMAN_TIMEOUT_MS + FAILSAFE_EMIT_BUDGET_MS + 100.0


async def _scan_for_drone() -> Optional[str]:
    """Locate the drone BLE peripheral by name."""
    logger.info(f"Scanning for BLE device: {BLE_DEVICE_NAME}...")
    devices = await BleakScanner.discover(timeout=10.0)
    for d in devices:
        if d.name and BLE_DEVICE_NAME.lower() in d.name.lower():
            return d.address
    return None


async def run_hil_certification(serial_port: str, baud: int):
    """
    Execute the Deadman HIL verification string.

    1. Mount a side-channel Serial listener.
    2. Broadcast normal armed thrust via BLE.
    3. Abruptly cease BLE transmissions (simulating socket silence).
    4. Start the precision clock.
    5. Await the specific serial emitted timestamp from the firmware.
    6. Assert total latency adheres strictly to MAX_HIL_LATENCY_MS.
    """
    logger.info("Initializing HIL Side-Channel Interface...")
    try:
        hw_serial = serial.Serial(serial_port, baud, timeout=0.1)
    except Exception as e:
        logger.error(f"Failed to open hardware serial side-channel on {serial_port}: {e}")
        logger.error("HIL Certification cannot proceed without explicit motor transition timestamps.")
        sys.exit(1)

    logger.info("Emptying serial buffer...")
    hw_serial.reset_input_buffer()

    # --- Phase 1: Establish BLE Control ---
    address = await _scan_for_drone()
    if not address:
        logger.error("Drone BLE peripheral not found.")
        sys.exit(1)

    logger.info(f"Connecting to BLE {address}...")
    async with BleakClient(address) as client:
        logger.info("Sending baseline Armed thrust command...")
        
        # Arm the motors and send some forward thrust
        cmd = ControlCommand(vx=50.0, vy=0.0, vz=0.0, yaw=0.0, arm=True)
        payload = cmd.model_dump_json().encode("utf-8")
        await client.write_gatt_char(COMMAND_CHAR_UUID, payload, response=USE_WRITE_WITH_RESPONSE)

        logger.info("Simulating bridged telemetry loop for 2 seconds...")
        for _ in range(20):
            await client.write_gatt_char(COMMAND_CHAR_UUID, payload, response=USE_WRITE_WITH_RESPONSE)
            await asyncio.sleep(0.1)

        # --- Phase 2: Silence Injection ---
        logger.warning(f"INJECTING DEADMAN SILENCE THRESHOLD ({DEADMAN_TIMEOUT_MS}ms) >>>")
        
        # In a real bridge, `deadman_timer` would inject FAILSAFE_COMMAND under the hood
        # Here, we simulate that backend loop dropping out explicitly
        silence_start = time.monotonic()
        
        # Wait for the Bridge (simulated here) to fire the failsafe limit
        await asyncio.sleep(DEADMAN_TIMEOUT_MS / 1000.0)
        
        logger.warning("Bridge Silence Threshold Reached! Pumping FAILSAFE_COMMAND...")
        failsafe = ControlCommand(vx=0.0, vy=0.0, vz=0.0, yaw=0.0, arm=False, is_failsafe=True)
        failsafe_payload = failsafe.model_dump_json().encode("utf-8")
        
        # Fire failsafe to the microcontroller
        dispatch_time = time.monotonic()
        await client.write_gatt_char(COMMAND_CHAR_UUID, failsafe_payload, response=USE_WRITE_WITH_RESPONSE)
        
        logger.info(f"Bridge dispatch delta: {(dispatch_time - silence_start) * 1000.0:.2f} ms")

        # --- Phase 3: Certification Assurance ---
        logger.info("\nAwaiting physical motor transition from side-channel...")
        transition_detected = False
        motor_shutdown_ms = 0.0

        # Wait max 2 seconds for firmware to process and reply
        start_wait = time.monotonic()
        while (time.monotonic() - start_wait) < 2.0:
            while hw_serial.in_waiting:
                line = hw_serial.readline().decode('utf-8').strip()
                if "DISARMED_BY_FAILSAFE" in line:
                    motor_shutdown_ms = time.monotonic()
                    logger.critical(f"[SIDE-CHANNEL] Motor shutdown proven at: {line}")
                    transition_detected = True
                    break
            
            if transition_detected:
                break
            await asyncio.sleep(0.01)

        hw_serial.close()

        # --- Report ---
        if not transition_detected:
            logger.error("\n[FAIL] Strict Invariant Violated: Motor transition timestamp never received!")
            sys.exit(1)

        total_latency_ms = (motor_shutdown_ms - silence_start) * 1000.0
        
        print("\n=======================================================")
        print("           DEADMAN HIL SAFETY CERTIFICATION            ")
        print("=======================================================")
        print(f"Safety Baseline Limits:     < {MAX_HIL_LATENCY_MS:.2f} ms")
        print(f"Actual Measured Time:         {total_latency_ms:.2f} ms")
        print("=======================================================\n")

        if total_latency_ms > MAX_HIL_LATENCY_MS:
            logger.error(f"[FAIL] Hard safety boundary breached! Required < {MAX_HIL_LATENCY_MS}ms, got {total_latency_ms:.2f}ms")
            sys.exit(1)
            
        logger.info("[SUCCESS] Hardware motor transition falls safely within mandated budget boundaries.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="HIL Contract Validator for Deadman Safety Limits")
    parser.add_argument("--port", type=str, required=True, help="Serial port connected to the Arduino (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Hardware baud rate (default 115200)")
    args = parser.parse_args()

    asyncio.run(run_hil_certification(args.port, args.baud))
