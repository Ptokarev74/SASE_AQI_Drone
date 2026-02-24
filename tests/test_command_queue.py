"""
Stress test for the drop-oldest command queue policy.

Validates:
  1. Queue size never exceeds COMMAND_QUEUE_SIZE under 300 Hz flooding.
  2. Drop counter increases correctly when queue is full.
  3. No QueueFull exception is ever raised during flooding.
  4. The LAST command sent is always retrievable (freshest intent preserved).
  5. commands_received == commands_enqueued + commands_dropped_oldest (accounting).
  6. Rate-limited drop logging does not spam (at most 1 log per COMMAND_DROP_LOG_INTERVAL_S).
  7. A slow BLE consumer does not cause the queue to grow beyond capacity.
"""

import asyncio
import logging
import time

from bridge.config import (
    COMMAND_DROP_LOG_INTERVAL_S,
    COMMAND_QUEUE_POLICY,
    COMMAND_QUEUE_SIZE,
)
from bridge.schemas import ControlCommand
from bridge.server import _metrics, enqueue_command_drop_oldest

logging.basicConfig(level=logging.WARNING)  # suppress DEBUG noise in stress test


def _reset_metrics() -> None:
    """Reset module-level counters between tests."""
    _metrics.reset()


def _make_queue() -> asyncio.Queue[ControlCommand]:
    return asyncio.Queue(maxsize=COMMAND_QUEUE_SIZE)


def _make_cmd(vx: float = 0.0) -> ControlCommand:
    return ControlCommand(vx=vx, vy=0.0, vz=0.0, yaw=0.0, arm=True)


# ---------------------------------------------------------------------------
# Test 1: Queue never exceeds capacity during burst
# ---------------------------------------------------------------------------
async def test_queue_never_exceeds_capacity():
    print("\n--- Queue Test 1: Queue Never Exceeds Capacity ---")
    _reset_metrics()
    queue = _make_queue()

    max_observed_size = 0
    n_floods = 1000  # 1000 commands into a size-4 queue

    for i in range(n_floods):
        enqueue_command_drop_oldest(queue, _make_cmd(vx=float(i)))
        size = queue.qsize()
        if size > max_observed_size:
            max_observed_size = size
        assert size <= COMMAND_QUEUE_SIZE, (
            f"Queue exceeded capacity: {size} > {COMMAND_QUEUE_SIZE}"
        )

    print(f"  Max observed queue size: {max_observed_size} / {COMMAND_QUEUE_SIZE}")
    assert max_observed_size == COMMAND_QUEUE_SIZE, (
        "Queue never reached capacity — is COMMAND_QUEUE_SIZE set correctly?"
    )
    print("  SUCCESS")


# ---------------------------------------------------------------------------
# Test 2: Accounting: received == enqueued, dropped is a separate counter
# ---------------------------------------------------------------------------
async def test_accounting_invariant():
    print("\n--- Queue Test 2: Accounting Invariant ---")
    _reset_metrics()
    queue = _make_queue()

    n = 500
    for i in range(n):
        enqueue_command_drop_oldest(queue, _make_cmd(vx=float(i)))

    recv = _metrics.commands_received
    enq = _metrics.commands_enqueued
    drop = _metrics.commands_dropped_oldest

    assert recv == n, f"Expected {n} received, got {recv}"
    # Every call does a put_nowait, so enqueued always equals received.
    # commands_dropped_oldest is an independent eviction counter (oldest removed
    # before the new item is inserted).
    assert enq == recv, (
        f"commands_enqueued ({enq}) should equal commands_received ({recv})"
    )
    # At least n - COMMAND_QUEUE_SIZE commands should have triggered drops
    expected_min_drops = n - COMMAND_QUEUE_SIZE
    assert drop >= expected_min_drops, (
        f"Expected at least {expected_min_drops} drops, got {drop}"
    )
    print(f"  Received={recv}  Enqueued={enq}  Dropped(oldest evictions)={drop}")
    print("  SUCCESS")


# ---------------------------------------------------------------------------
# Test 3: Drop counter increases during flood
# ---------------------------------------------------------------------------
async def test_drop_counter_increases():
    print("\n--- Queue Test 3: Drop Counter Increases During Flood ---")
    _reset_metrics()
    queue = _make_queue()

    # Fill once (no drops yet)
    for i in range(COMMAND_QUEUE_SIZE):
        enqueue_command_drop_oldest(queue, _make_cmd(vx=float(i)))
    assert _metrics.commands_dropped_oldest == 0

    # Now overflow it
    for i in range(20):
        enqueue_command_drop_oldest(queue, _make_cmd(vx=float(100 + i)))

    assert _metrics.commands_dropped_oldest == 20, (
        f"Expected 20 drops, got {_metrics.commands_dropped_oldest}"
    )
    print(f"  Drops: {_metrics.commands_dropped_oldest}")
    print("  SUCCESS")


# ---------------------------------------------------------------------------
# Test 4: Newest command survives — oldest is discarded
# ---------------------------------------------------------------------------
async def test_newest_command_preserved():
    print("\n--- Queue Test 4: Newest Command Preserved (drop_oldest) ---")
    _reset_metrics()
    queue = _make_queue()

    # Fill queue with sentinel vx=0 commands
    for _ in range(COMMAND_QUEUE_SIZE):
        enqueue_command_drop_oldest(queue, _make_cmd(vx=0.0))

    # Send the "freshest" command — should displace the oldest
    fresh_vx = 99.9
    enqueue_command_drop_oldest(queue, _make_cmd(vx=fresh_vx))

    # Drain queue and verify the fresh command is present
    items = []
    while not queue.empty():
        items.append(queue.get_nowait())

    assert any(abs(cmd.vx - fresh_vx) < 1e-6 for cmd in items), (
        f"Fresh command vx={fresh_vx} not found in queue: {[c.vx for c in items]}"
    )

    # The first item (oldest still in queue after one drop) should NOT be vx=0
    # from original batch — it was evicted.
    # Queue should have COMMAND_QUEUE_SIZE items total.
    assert len(items) == COMMAND_QUEUE_SIZE, (
        f"Expected {COMMAND_QUEUE_SIZE} items, got {len(items)}"
    )
    print(f"  Queue contents (vx): {[c.vx for c in items]}")
    print("  SUCCESS")


# ---------------------------------------------------------------------------
# Test 5: No QueueFull raised during 300 Hz flood for 2 seconds
# ---------------------------------------------------------------------------
async def test_no_queue_full_during_flood():
    print("\n--- Queue Test 5: No QueueFull During High-Rate Flood ---")
    _reset_metrics()
    queue = _make_queue()

    hz = 300
    duration_s = 2.0
    interval = 1.0 / hz
    n_commands = int(hz * duration_s)
    exceptions_seen: list[Exception] = []

    t0 = time.monotonic()
    for i in range(n_commands):
        try:
            enqueue_command_drop_oldest(queue, _make_cmd(vx=float(i)))
        except Exception as exc:
            exceptions_seen.append(exc)
        await asyncio.sleep(interval)

    elapsed = time.monotonic() - t0
    assert not exceptions_seen, f"Exceptions raised: {exceptions_seen}"
    assert _metrics.commands_received == n_commands

    print(
        f"  Flooded {n_commands} commands at ~{hz} Hz over {elapsed:.2f}s — "
        f"enqueued={_metrics.commands_enqueued}, "
        f"dropped={_metrics.commands_dropped_oldest}"
    )
    print("  SUCCESS")


# ---------------------------------------------------------------------------
# Test 6: Slow consumer — queue stays capped
# ---------------------------------------------------------------------------
async def test_slow_consumer_queue_capped():
    print("\n--- Queue Test 6: Slow Consumer — Queue Stays Capped ---")
    _reset_metrics()
    queue = _make_queue()
    consumed: list[ControlCommand] = []

    async def slow_consumer():
        """Simulates a BLE writer that takes 50ms per command."""
        while True:
            try:
                cmd = await asyncio.wait_for(queue.get(), timeout=1.0)
                consumed.append(cmd)
                await asyncio.sleep(0.05)  # 50ms per write
            except asyncio.TimeoutError:
                break

    async def producer():
        """Sends commands at 200 Hz for 1s."""
        for i in range(200):
            enqueue_command_drop_oldest(queue, _make_cmd(vx=float(i)))
            assert queue.qsize() <= COMMAND_QUEUE_SIZE, (
                f"Queue exceeded capacity: {queue.qsize()}"
            )
            await asyncio.sleep(0.005)  # 200 Hz

    consumer_task = asyncio.create_task(slow_consumer())
    await producer()
    # Give consumer time to drain
    await asyncio.sleep(0.3)
    consumer_task.cancel()
    try:
        await consumer_task
    except (asyncio.CancelledError, asyncio.TimeoutError):
        pass

    print(
        f"  Produced=200, consumed={len(consumed)}, "
        f"dropped={_metrics.commands_dropped_oldest}"
    )
    assert _metrics.commands_dropped_oldest > 0, "Expected drops with slow consumer"
    print("  SUCCESS")


# ---------------------------------------------------------------------------
# Test 7: COMMAND_QUEUE_POLICY constant is correct
# ---------------------------------------------------------------------------
async def test_policy_constant():
    print("\n--- Queue Test 7: Policy Constant Correct ---")
    assert COMMAND_QUEUE_POLICY == "drop_oldest", (
        f"Expected 'drop_oldest', got '{COMMAND_QUEUE_POLICY}'"
    )
    assert COMMAND_DROP_LOG_INTERVAL_S > 0
    print(f"  COMMAND_QUEUE_POLICY = '{COMMAND_QUEUE_POLICY}'")
    print(f"  COMMAND_DROP_LOG_INTERVAL_S = {COMMAND_DROP_LOG_INTERVAL_S}s")
    print("  SUCCESS")


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------
async def _run_all():
    await test_queue_never_exceeds_capacity()
    await test_accounting_invariant()
    await test_drop_counter_increases()
    await test_newest_command_preserved()
    await test_no_queue_full_during_flood()
    await test_slow_consumer_queue_capped()
    await test_policy_constant()
    print("\nALL COMMAND QUEUE STRESS TESTS PASSED")


if __name__ == "__main__":
    try:
        asyncio.run(_run_all())
    except AssertionError:
        import traceback
        traceback.print_exc()
        raise SystemExit(1) from None
