#!/usr/bin/env python3
"""
Performance Qualification Suite for SASE AQI Drone Bridge.

This standalone script enforces strict hardware qualification budgets.
Unlike CI functional tests, this script MUST be run on deterministic, dedicated
hardware (e.g. pinned CPU frequencies, isolated networking).

Workload Definition:
  1 Command = 1 Injection into the WebSocket intake + 1 Consumer Dispatch to BLE.
  Latency is measured monotonically from ts_received → ts_dispatched.

Overload Policy:
  Under burst conditions exceeding queue capacity, the system evicts the OLDEST
  commands (drop-oldest) to prioritize FRESHNESS. Stale commands are never
  executed. This is intentional, controlled degradation — not data loss.
  Queue Capacity: COMMAND_QUEUE_SIZE  |  Max Command Age: MAX_COMMAND_AGE_MS

It validates (all budgets scoped to a 30Hz / ~33ms control period):
  1. Mean E2E latency      < 5ms
  2. P95 E2E latency       < 10ms
  3. P99 E2E latency       < 10ms   (< 1/3 of the 33ms control period)
  4. Absolute Worst Case   < 20ms   (< 2/3 of the 33ms control period)
  5. Maximum E2E Jitter    < 10ms
  6. No single command     > 33ms   (CONTROL_PERIOD_MS — zero deadline misses)
  7. Freshness invariant   = 0 stale commands executed

Validated bounded latency under defined workload and hardware profile.
NOT a claim of hard real-time scheduling guarantees.

Usage:
    python -m tests.test_perf_qualify
"""

import asyncio
import gc
import statistics
import sys
from unittest.mock import AsyncMock

from aqi_bridge.api import _handle_control_message, _metrics
from aqi_bridge.app import command_consumer_loop
from aqi_bridge.ble import BLEDroneClient
from aqi_bridge.config import COMMAND_QUEUE_SIZE, MAX_COMMAND_AGE_MS
from aqi_bridge.models import ControlCommand

# ---------------------------------------------------------------------------
# Latency Budgets (aligned to 30Hz / ~33ms control period)
# ---------------------------------------------------------------------------
CONTROL_PERIOD_MS = 33.3

MAX_MEAN_LATENCY_MS = 5.0
MAX_P95_LATENCY_MS = 10.0
MAX_P99_LATENCY_MS = 10.0
MAX_WORST_CASE_MS = 20.0
MAX_JITTER_MS = 10.0

SPIKE_TRACE_THRESHOLD_MS = 5.0

# ---------------------------------------------------------------------------
# Workload Definition
# ---------------------------------------------------------------------------
TARGET_COMMAND_COUNT = 1000
BURST_FLOOD_SIZE = COMMAND_QUEUE_SIZE + 50
BURST_COMMANDS = BURST_FLOOD_SIZE
STEADY_STATE_COMMANDS = TARGET_COMMAND_COUNT - BURST_COMMANDS
TOTAL_COMMAND_WORKLOAD = STEADY_STATE_COMMANDS + BURST_COMMANDS

INJECTION_BATCH_SIZE = 10


async def run_qualification():
    print("=" * 60)
    print("Performance Qualification Suite")
    print("Workload Definition: 1 Command = 1 Injection + 1 Dispatch")
    print(f"Target Workload:     {TOTAL_COMMAND_WORKLOAD} Commands")
    print(f"Control Period:      {CONTROL_PERIOD_MS:.1f} ms (30 Hz)")
    print()
    print("Overload Policy: drop-oldest (intentional freshness preservation)")
    print(f"  Queue Capacity:  {COMMAND_QUEUE_SIZE} commands")
    print(f"  Max Command Age: {MAX_COMMAND_AGE_MS:.0f} ms (stale commands rejected at dequeue)")
    print("  Guarantee:       Oldest commands evicted under burst; fresh commands prioritized.")
    print("=" * 60)

    _metrics.reset()

    ble = AsyncMock(spec=BLEDroneClient)
    ble.connected = asyncio.Event()
    ble.connected.set()
    ble._write_command_bytes = AsyncMock(return_value=True)

    queue = asyncio.Queue(maxsize=COMMAND_QUEUE_SIZE)
    last_cmd_time = [0.0]
    write_metrics = {
        "success": 0,
        "errors": 0,
        "dropped": 0,
        "dropped_stale": 0,
        "last_error_timestamp": 0.0,
        "max_command_age_ms": 0.0,
        "sum_command_age_ms": 0.0,
        "total_commands_checked": 0,
    }

    # GC instrumentation
    gc.collect()
    gc_counts_before = gc.get_count()

    consumer_task = asyncio.create_task(
        command_consumer_loop(ble, queue, last_cmd_time, write_metrics)
    )

    # Phase 1: Steady State — batch inject (no per-command sleep)
    print(f"\nPhase 1: Steady State ({STEADY_STATE_COMMANDS} commands, batch={INJECTION_BATCH_SIZE})")
    injected = 0
    while injected < STEADY_STATE_COMMANDS:
        batch = min(INJECTION_BATCH_SIZE, STEADY_STATE_COMMANDS - injected)
        for _ in range(batch):
            _handle_control_message('{"vx": 1.0, "yaw": 0.5, "arm": true}', queue)
            injected += 1
        await asyncio.sleep(0)

    # Phase 2: Burst Flood — intentional overload stress test
    print(f"Phase 2: Burst Flood  ({BURST_COMMANDS} commands) [intentional queue overload stress test]")
    for _ in range(BURST_COMMANDS):
        _handle_control_message('{"vx": 0.0, "yaw": 0.0, "arm": false}', queue)

    await asyncio.sleep(0.5)
    remaining_in_queue = queue.qsize()

    consumer_task.cancel()
    try:
        await consumer_task
    except asyncio.CancelledError:
        pass

    gc_counts_after = gc.get_count()
    gc_delta = tuple(gc_counts_after[i] - gc_counts_before[i] for i in range(3))

    # Extract latency data and freshness information in time order
    raw_calls = ble._write_command_bytes.call_args_list
    records = []
    spike_traces = []
    stale_violations = 0  # commands that reached BLE but exceeded MAX_COMMAND_AGE_MS

    for call in raw_calls:
        cmd: ControlCommand = call[0][0]
        e2e_ms = (cmd.ts_dispatched - cmd.ts_received) * 1000.0
        seg_enq_ms = (cmd.ts_enqueued - cmd.ts_received) * 1000.0
        seg_queue_ms = (cmd.ts_dequeued - cmd.ts_enqueued) * 1000.0
        seg_prewrite_ms = (cmd.ts_write_started - cmd.ts_dequeued) * 1000.0 if cmd.ts_write_started else 0.0
        seg_write_ms = (cmd.ts_dispatched - (cmd.ts_write_started or cmd.ts_dequeued)) * 1000.0

        # Freshness invariant check: no command should reach BLE if age > MAX_COMMAND_AGE_MS
        age_at_dispatch_ms = (cmd.ts_dispatched - cmd.ts_received) * 1000.0
        if age_at_dispatch_ms > MAX_COMMAND_AGE_MS and not getattr(cmd, "is_failsafe", False):
            stale_violations += 1

        records.append({
            "e2e_ms": e2e_ms,
            "ts_received": cmd.ts_received,
            "seg_enq_ms": seg_enq_ms,
            "seg_queue_ms": seg_queue_ms,
            "seg_prewrite_ms": seg_prewrite_ms,
            "seg_write_ms": seg_write_ms,
            "age_at_dispatch_ms": age_at_dispatch_ms,
        })

        if e2e_ms > SPIKE_TRACE_THRESHOLD_MS:
            spike_traces.append({
                "e2e_ms": e2e_ms,
                "seg_enq_ms": seg_enq_ms,
                "seg_queue_ms": seg_queue_ms,
                "seg_prewrite_ms": seg_prewrite_ms,
                "seg_write_ms": seg_write_ms,
            })

    count = len(records)
    latencies_ms = [r["e2e_ms"] for r in records]

    # Max age seen at BLE write dispatch
    max_age_at_dispatch = max((r["age_at_dispatch_ms"] for r in records), default=0.0)

    # ---------------------------------------------------------------------------
    # Workload Accounting
    # ---------------------------------------------------------------------------
    generated = _metrics.commands_received
    evicted_queue = _metrics.commands_dropped_oldest
    written = count
    errors = write_metrics["errors"]
    stale_drops = write_metrics["dropped_stale"]
    total_accounted = written + stale_drops + errors + evicted_queue + remaining_in_queue
    eviction_pct = (evicted_queue / generated * 100.0) if generated > 0 else 0.0

    print("\n--- Workload Accounting (Unit: Commands) ---")
    print(f"Commands Requested: {TOTAL_COMMAND_WORKLOAD}")
    print(f"Commands Generated: {generated}")
    print("---------------------------")
    print(f"  + Written to BLE: {written}")
    print(f"  + Left in Queue:  {remaining_in_queue}")
    print(f"  + Stale Dropped:  {stale_drops}")
    print(f"  + Queue Evicted:  {evicted_queue}")
    print(f"  + Write Errors:   {errors}")
    print("---------------------------")
    print(f"Total Accounted:    {total_accounted}")
    print(f"Delta (Unaccounted): {TOTAL_COMMAND_WORKLOAD - total_accounted}")
    print(f"GC Pauses (gen 0/1/2): {gc_delta[0]}/{gc_delta[1]}/{gc_delta[2]}")

    if generated != TOTAL_COMMAND_WORKLOAD:
        print(f"\nFAIL: Workload mismatch. Requested {TOTAL_COMMAND_WORKLOAD}, generated {generated}.")
        sys.exit(1)

    if TOTAL_COMMAND_WORKLOAD != total_accounted:
        print(f"\nFAIL: Zero-loss accounting failed. {TOTAL_COMMAND_WORKLOAD - total_accounted} commands unaccounted.")
        sys.exit(1)

    if not latencies_ms:
        print("\nFAIL: No latency data collected. Consumer failure.")
        sys.exit(1)

    # ---------------------------------------------------------------------------
    # Burst Behavior Summary
    # ---------------------------------------------------------------------------
    freshness_preserved = stale_violations == 0
    print("\n--- Burst Behavior Summary ---")
    print(f"Burst Size:              {BURST_COMMANDS} commands (intentional overload stress test)")
    print(f"Queue Capacity:          {COMMAND_QUEUE_SIZE} commands")
    print(f"Commands Evicted:        {evicted_queue} ({eviction_pct:.1f}% of total workload)")
    print(f"Freshness Preserved:     {'YES' if freshness_preserved else 'NO — VIOLATION'}")
    print(f"Max Age at BLE Write:    {max_age_at_dispatch:.2f} ms (limit: {MAX_COMMAND_AGE_MS:.0f} ms)")
    print(f"Stale Commands Executed: {stale_violations}")
    if freshness_preserved:
        print("Freshness invariant maintained: 0 stale commands executed.")
        print("Burst flood intentionally exceeded queue capacity; oldest commands were evicted")
        print("to preserve real-time responsiveness. No stale commands were executed.")
    else:
        print(f"FAIL: {stale_violations} stale command(s) reached the BLE write path!")

    # Freshness invariant — hard failure
    if not freshness_preserved:
        print(f"\nFAIL: Freshness invariant violated. {stale_violations} stale command(s) reached BLE.")
        sys.exit(1)

    # ---------------------------------------------------------------------------
    # Spike Trace Summary
    # ---------------------------------------------------------------------------
    if spike_traces:
        print(f"\n--- Spike Trace ({len(spike_traces)} outliers above {SPIKE_TRACE_THRESHOLD_MS:.0f}ms) ---")
        print(f"{'E2E':>8}  {'Enqueue':>8}  {'In Queue':>9}  {'Pre-Write':>10}  {'Write':>7}")
        for s in sorted(spike_traces, key=lambda x: x["e2e_ms"], reverse=True)[:10]:
            print(f"{s['e2e_ms']:>7.2f}ms  {s['seg_enq_ms']:>6.2f}ms  {s['seg_queue_ms']:>8.2f}ms  "
                  f"{s['seg_prewrite_ms']:>9.2f}ms  {s['seg_write_ms']:>6.2f}ms")
    else:
        print(f"\nSpike Trace: No spikes above {SPIKE_TRACE_THRESHOLD_MS:.0f}ms threshold.")

    # ---------------------------------------------------------------------------
    # Statistical Math (time-ordered jitter)
    # ---------------------------------------------------------------------------
    sorted_latencies = sorted(latencies_ms)
    mean = statistics.mean(latencies_ms)
    p95 = sorted_latencies[int(count * 0.95)]
    p99 = sorted_latencies[int(count * 0.99)]
    worst = sorted_latencies[-1]
    jitter_diffs = [abs(latencies_ms[i] - latencies_ms[i - 1]) for i in range(1, count)]
    max_jitter = max(jitter_diffs) if jitter_diffs else 0.0
    deadline_misses = sum(1 for x in latencies_ms if x > CONTROL_PERIOD_MS)

    print("\n--- Latency Statistics ---")
    print(f"{'Metric':<25}  {'Measured':>10}  {'Limit':>10}  Status")
    print(f"{'-'*58}")

    def row(label, val, limit, unit="ms"):
        status = "PASS" if val <= limit else "FAIL"
        return f"{label:<25}  {val:>9.2f}{unit}  {limit:>9.2f}{unit}  {status}"

    print(row("Mean Latency", mean, MAX_MEAN_LATENCY_MS))
    print(row("P95 Latency", p95, MAX_P95_LATENCY_MS))
    print(row("P99 Latency", p99, MAX_P99_LATENCY_MS))
    print(row("Worst Case", worst, MAX_WORST_CASE_MS))
    print(row("Max Jitter", max_jitter, MAX_JITTER_MS))
    miss_status = "PASS" if deadline_misses == 0 else "FAIL"
    print(f"{'Deadline Misses':<25}  {deadline_misses:>9}     {0:>9}     {miss_status}")
    print(f"{'-'*58}")

    # ---------------------------------------------------------------------------
    # Pass / Fail Verdict
    # ---------------------------------------------------------------------------
    failed = False
    if mean > MAX_MEAN_LATENCY_MS:
        print(f"FAIL: Mean ({mean:.2f}ms) exceeds budget ({MAX_MEAN_LATENCY_MS}ms)")
        failed = True
    if p95 > MAX_P95_LATENCY_MS:
        print(f"FAIL: P95 ({p95:.2f}ms) exceeds budget ({MAX_P95_LATENCY_MS}ms)")
        failed = True
    if p99 > MAX_P99_LATENCY_MS:
        print(f"FAIL: P99 ({p99:.2f}ms) exceeds budget ({MAX_P99_LATENCY_MS}ms)")
        failed = True
    if worst > MAX_WORST_CASE_MS:
        print(f"FAIL: Worst case ({worst:.2f}ms) exceeds budget ({MAX_WORST_CASE_MS}ms)")
        failed = True
    if max_jitter > MAX_JITTER_MS:
        print(f"FAIL: Max Jitter ({max_jitter:.2f}ms) exceeds budget ({MAX_JITTER_MS}ms)")
        failed = True
    if deadline_misses > 0:
        print(f"FAIL: {deadline_misses} command(s) missed the {CONTROL_PERIOD_MS:.1f}ms deadline")
        failed = True

    if failed:
        print("\nQUALIFICATION FAILED.")
        print("Validated bounded latency was NOT confirmed under this workload and hardware profile.")
        sys.exit(1)
    else:
        print("\nQUALIFICATION PASSED.")
        print("Validated bounded latency confirmed under defined workload and hardware profile.")
        sys.exit(0)


if __name__ == "__main__":
    if sys.platform == "win32":
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

    asyncio.run(run_qualification())
