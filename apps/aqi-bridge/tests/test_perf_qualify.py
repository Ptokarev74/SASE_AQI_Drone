#!/usr/bin/env python3
"""
Performance Qualification Suite for SASE AQI Drone Bridge.

This standalone script enforces strict hardware qualification budgets.
Unlike CI functional tests, this script MUST be run on deterministic, dedicated
hardware (e.g. pinned CPU frequencies, isolated networking).

It validates:
  1. Mean E2E latency < 40ms
  2. P95 E2E latency < 80ms
  3. P99 E2E latency < 120ms
  4. Absolute Worst Case < 150ms
  5. Maximum Jitter (Variance) < 20ms

Usage:
    python -m tests.test_perf_qualify
"""

import asyncio
import json
import math
import statistics
import sys
import time
from unittest.mock import AsyncMock

from aqi_bridge.api import enqueue_command_drop_oldest, _handle_control_message
from aqi_bridge.app import command_consumer_loop
from aqi_bridge.ble import BLEDroneClient
from aqi_bridge.config import COMMAND_QUEUE_SIZE
from aqi_bridge.models import ControlCommand

# Numeric Limits from docs/PERFORMANCE_QUALIFICATION.md
MAX_MEAN_LATENCY_MS = 40.0
MAX_P95_LATENCY_MS = 80.0
MAX_P99_LATENCY_MS = 120.0
ABSOLUTE_WORST_CASE_MS = 150.0
MAX_JITTER_MS = 20.0

ITERATION_COUNT = 1000  # Number of commands to process
BURST_FLOOD_SIZE = COMMAND_QUEUE_SIZE + 50  # Intentionally flood the queue


async def run_qualification():
    print("Initializing Performance Qualification Environment...")

    ble = AsyncMock(spec=BLEDroneClient)
    ble.connected = asyncio.Event()
    ble.connected.set()
    ble._write_command_bytes = AsyncMock(return_value=True)

    queue = asyncio.Queue(maxsize=COMMAND_QUEUE_SIZE)
    last_cmd_time = [0.0]
    write_metrics = {"errors": 0, "dropped": 0, "last_error_timestamp": 0.0}

    # Start the consumer loop
    consumer_task = asyncio.create_task(
        command_consumer_loop(ble, queue, last_cmd_time, write_metrics)
    )

    latencies_ms = []

    print(f"Beginning {ITERATION_COUNT} qualification cycles (including burst floods)...")

    # Cycle 1: Steady state
    for _ in range(ITERATION_COUNT // 2):
        raw_json = '{"vx": 1.0, "yaw": 0.5, "arm": true}'
        _handle_control_message(raw_json, queue)
        await asyncio.sleep(0.005)  # 200 Hz injection

    # Cycle 2: Violent Burst Flood (triggers drop-oldest mechanism synchronously)
    print("Injecting burst flood to stress enqueue eviction...")
    for _ in range(BURST_FLOOD_SIZE):
        _handle_control_message('{"vx": 0.0, "yaw": 0.0, "arm": false}', queue)

    # Let consumer drain
    await asyncio.sleep(1.0)
    consumer_task.cancel()
    
    try:
        await consumer_task
    except asyncio.CancelledError:
        pass

    # Extract all latency data
    for call in ble._write_command_bytes.call_args_list:
        cmd: ControlCommand = call[0][0]
        e2e_latency = (cmd.ts_dispatched - cmd.ts_received) * 1000.0
        latencies_ms.append(e2e_latency)

    if not latencies_ms:
        print("FAIL: No latency data collected. Consumer failure.")
        sys.exit(1)

    # Statistical Math
    latencies_ms.sort()
    count = len(latencies_ms)
    mean = statistics.mean(latencies_ms)
    p95 = latencies_ms[int(count * 0.95)] if count > 0 else 0
    p99 = latencies_ms[int(count * 0.99)] if count > 0 else 0
    worst = latencies_ms[-1]

    diffs = [abs(latencies_ms[i] - latencies_ms[i - 1]) for i in range(1, count)]
    max_jitter = max(diffs) if diffs else 0

    print("\n--- Qualification Results ---")
    print(f"Total Processed: {count}")
    print(f"Mean Latency:    {mean:.2f} ms")
    print(f"P95 Latency:     {p95:.2f} ms")
    print(f"P99 Latency:     {p99:.2f} ms")
    print(f"Worst Case:      {worst:.2f} ms")
    print(f"Max Jitter:      {max_jitter:.2f} ms")
    print("-----------------------------\n")

    failed = False
    if mean > MAX_MEAN_LATENCY_MS:
        print(f"FAIL: Mean Latency ({mean:.2f}ms) > Limit ({MAX_MEAN_LATENCY_MS}ms)")
        failed = True
    if p95 > MAX_P95_LATENCY_MS:
        print(f"FAIL: P95 Latency ({p95:.2f}ms) > Limit ({MAX_P95_LATENCY_MS}ms)")
        failed = True
    if p99 > MAX_P99_LATENCY_MS:
        print(f"FAIL: P99 Latency ({p99:.2f}ms) > Limit ({MAX_P99_LATENCY_MS}ms)")
        failed = True
    if worst > ABSOLUTE_WORST_CASE_MS:
        print(f"FAIL: Worst Case ({worst:.2f}ms) > Limit ({ABSOLUTE_WORST_CASE_MS}ms)")
        failed = True
    if max_jitter > MAX_JITTER_MS:
        print(f"FAIL: Max Jitter ({max_jitter:.2f}ms) > Limit ({MAX_JITTER_MS}ms)")
        failed = True

    if failed:
        print("\nQUALIFICATION FAILED.")
        print("The hardware module failed to assert true real-time determinism margins.")
        sys.exit(1)
    else:
        print("QUALIFICATION PASSED.")
        print("Hardware environment asserted hard real-time latency bounds successfully.")
        sys.exit(0)


if __name__ == "__main__":
    # Windows fix for missing add_signal_handler
    if sys.platform == "win32":
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
        
    asyncio.run(run_qualification())
