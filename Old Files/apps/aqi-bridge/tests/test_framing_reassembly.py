"""
Verification script for the BLE telemetry reassembly mechanism.

Simulates fragmented, bunched, and malformed BLE notifications to ensure
the reassembly buffer in BLEDroneClient is robust.
"""

import logging
import zlib

from aqi_bridge.ble import BLEDroneClient

# Setup minimal logging to see frame extractions
logging.basicConfig(level=logging.DEBUG)

def _with_crc(payload: bytes) -> bytes:
    """Helper to append CRC32 to a payload."""
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    return payload + f"|{crc:08x}\n".encode("ascii")

def test_reassembly():
    client = BLEDroneClient()
    
    # 1. Test case: Full message in one go
    print("\n--- Test 1: Full Message ---")
    msg1 = _with_crc(b'{"status": "ok", "aqi": 10}')
    client._on_telemetry_notification(0, bytearray(msg1))
    assert client.latest_telemetry is not None
    assert client.latest_telemetry.status == "ok"
    assert client.latest_telemetry.aqi == 10
    print("SUCCESS")

    # 2. Test case: Fragmented message (2 parts)
    print("\n--- Test 2: Fragmented Message ---")
    client.latest_telemetry = None
    full_msg = _with_crc(b'{"status": "fragmented", "aqi": 20}')
    part1 = full_msg[:15]
    part2 = full_msg[15:]
    client._on_telemetry_notification(0, bytearray(part1))
    assert client.latest_telemetry is None  # Should still be waiting
    client._on_telemetry_notification(0, bytearray(part2))
    assert client.latest_telemetry.status == "fragmented"
    assert client.latest_telemetry.aqi == 20
    print("SUCCESS")

    # 3. Test case: Bunched messages (2 in 1 notification)
    print("\n--- Test 3: Bunched Messages ---")
    bunched = _with_crc(b'{"status": "first"}') + _with_crc(b'{"status": "second"}')
    client._on_telemetry_notification(0, bytearray(bunched))
    assert client.latest_telemetry.status == "second"
    print("SUCCESS")

    # 4. Test case: Malformed followed by valid
    print("\n--- Test 4: Malformed + Valid ---")
    # A frame with missing separator should be dropped by CRC check
    f1 = b'{"status": "bad" broken}\n'
    f2 = _with_crc(b'{"status": "fixed"}')
    malformed = f1 + f2
    client._on_telemetry_notification(0, bytearray(malformed))
    assert client.latest_telemetry is not None
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
    msg_utf8 = _with_crc(b'{"status": "\xc2\xb0"}')
    part_a = msg_utf8[:15]
    part_b = msg_utf8[15:]
    client._on_telemetry_notification(0, bytearray(part_a))
    client._on_telemetry_notification(0, bytearray(part_b))
    assert client.latest_telemetry.status == "°"
    print("SUCCESS")

if __name__ == "__main__":
    try:
        test_reassembly()
        print("\nALL REASSEMBLY TESTS PASSED")
    except AssertionError:
        print("\nTEST FAILED")
        import traceback
        traceback.print_exc()
