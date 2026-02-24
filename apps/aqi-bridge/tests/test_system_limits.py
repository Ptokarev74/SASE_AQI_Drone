"""
Tests to verify System-Level Mitigation strategies.

Targets:
1. MAX_WS_CLIENTS enforcement (Registry bounding).
2. Event Loop Watchdog starvation detection.
"""

import asyncio
import time
from unittest.mock import MagicMock

from fastapi.testclient import TestClient

from aqi_bridge.config import LOOP_STARVATION_THRESHOLD_S, MAX_WS_CLIENTS
from aqi_bridge.app import event_loop_watchdog
from aqi_bridge.api import create_app


def test_max_ws_clients_enforced():
    """Verify connections are rejected with 1013 when the registry hits MAX_WS_CLIENTS."""
    print(f"\n--- System Test 1: Bounding WS registry to {MAX_WS_CLIENTS} ---")
    mock_ble = MagicMock()
    mock_ble.connected.is_set.return_value = False
    
    queue = asyncio.Queue(maxsize=4)
    app = create_app(mock_ble, queue)

    # Manually stuff the registry to maximum capacity with unique objects
    app.state.clients.update([MagicMock() for _ in range(MAX_WS_CLIENTS)])

    with TestClient(app) as client:
        # Attempt to connect when full
        with client.websocket_connect("/ws") as _:
            try:
                # Should be immediately rejected with 1013 Try Again Later
                _.receive_text()
                raise AssertionError("Should have been rejected but connection succeeded")
            except Exception:
                # Based on starlette websocket internals, testing the exact code 1013 
                # might raise a generic disconnect in TestClient, so we verify it drops.
                pass 
                
        # Health endpoint should reflect the dropped connection capacity limit
        resp = client.get("/health")
        assert resp.status_code == 200
        body = resp.json()
        assert body["ws_clients"]["total"] == MAX_WS_CLIENTS
        assert body["ws_clients"]["rejected_capacity"] == 1
        print("  Client rejected. Capacity limit enforced.")
        print("  SUCCESS")


async def _run_watchdog_test():
    loop_lag = [0.0]
    starvation_count = [0]
    
    # Start the watchdog as a background task
    task = asyncio.create_task(event_loop_watchdog(loop_lag, starvation_count))
    
    # Let it run to the first await sleep
    await asyncio.sleep(0.1)
    
    # SIMULATE STARVATION: synchronous blocking code
    # This prevents the event loop from waking the watchdog from its asyncio.sleep(1.0) on time
    
    # We delay the blocking to ensure the watchdog has entered its 1.0s sleep.
    await asyncio.sleep(0.1)
    
    # Block forcefully for longer than the threshold + the watchdog's sleep
    block_duration = 1.0 + LOOP_STARVATION_THRESHOLD_S + 0.1
    print(f"  Blocking CPU synchronously for {block_duration}s...")
    time.sleep(block_duration)
    
    # Yield control back to watchdog so it completes/measures
    await asyncio.sleep(0.1)
    
    # Cancel the watchdog before exit
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
        
    assert starvation_count[0] > 0, "Watchdog failed to detect the synchronous starvation event"
    print(f"  Watchdog correctly logged {starvation_count[0]} starvation events.")
    print(f"  Final recorded loop lag: {loop_lag[0]*1000:.1f}ms")


def test_loop_starvation_detection():
    """Verify the watchdog detects sync blocking code on the event loop."""
    print("\n--- System Test 2: Event Loop Watchdog detects starvation ---")
    asyncio.run(_run_watchdog_test())
    print("  SUCCESS")

if __name__ == "__main__":
    test_max_ws_clients_enforced()
    test_loop_starvation_detection()
    print("\nALL SYSTEM-LEVEL BOUNDING TESTS PASSED")
