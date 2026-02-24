"""
Verification script for the BLE telemetry reassembly mechanism.

Simulates fragmented, bunched, and malformed BLE notifications to ensure
the reassembly buffer in BLEDroneClient is robust.
"""

import json
import logging
from bridge.ble_client import BLEDroneClient
from bridge.schemas import TelemetryMessage

# Setup minimal logging to see frame extractions
logging.basicConfig(level=logging.DEBUG)

def test_reassembly():
    client = BLEDroneClient()
    
    # 1. Test case: Full message in one go
    print("\n--- Test 1: Full Message ---")
    msg1 = b'{"status": "ok", "aqi": 10}\n'
    client._on_telemetry_notification(0, bytearray(msg1))
    assert client.latest_telemetry is not None
    assert client.latest_telemetry.status == "ok"
    assert client.latest_telemetry.aqi == 10
    print("SUCCESS")

    # 2. Test case: Fragmented message (2 parts)
    print("\n--- Test 2: Fragmented Message ---")
    client.latest_telemetry = None
    part1 = b'{"status": "fragmented", '
    part2 = b'"aqi": 20}\n'
    client._on_telemetry_notification(0, bytearray(part1))
    assert client.latest_telemetry is None  # Should still be waiting
    client._on_telemetry_notification(0, bytearray(part2))
    assert client.latest_telemetry.status == "fragmented"
    assert client.latest_telemetry.aqi == 20
    print("SUCCESS")

    # 3. Test case: Bunched messages (2 in 1 notification)
    print("\n--- Test 3: Bunched Messages ---")
    bunched = b'{"status": "first"}\n{"status": "second"}\n'
    client._on_telemetry_notification(0, bytearray(bunched))
    assert client.latest_telemetry.status == "second"
    print("SUCCESS")

    # 4. Test case: Malformed followed by valid
    print("\n--- Test 4: Malformed + Valid ---")
    malformed = b'{"status": "bad" broken}\n{"status": "fixed"}\n'
    client._on_telemetry_notification(0, bytearray(malformed))
    assert client.latest_telemetry.status == "fixed"
    print("SUCCESS")

    # 5. Test case: Overflow guard
    print("\n--- Test 5: Overflow Guard ---")
    client._buffer = b""
    # Send a long stream of data without a newline
    junk = b"A" * 5000 
    client._on_telemetry_notification(0, bytearray(junk))
    assert len(client._buffer) == 0  # Should have been reset
    print("SUCCESS")

    # 6. Test case: UTF-8 partial (fragmenting a multibyte character)
    # The degree symbol ° is C2 B0 in UTF-8
    print("\n--- Test 6: Partial UTF-8 Character ---")
    part_a = b'{"status": "' + b'\xc2'
    part_b = b'\xb0"}\n'
    client._on_telemetry_notification(0, bytearray(part_a))
    client._on_telemetry_notification(0, bytearray(part_b))
    assert client.latest_telemetry.status == "°"
    print("SUCCESS")

if __name__ == "__main__":
    try:
        test_reassembly()
        print("\nALL REASSEMBLY TESTS PASSED")
    except AssertionError as e:
        print(f"\nTEST FAILED")
        import traceback
        traceback.print_exc()
