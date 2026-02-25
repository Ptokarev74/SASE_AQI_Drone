"""
Tests for the MTU chunked framing protocol (Layer 2).

Verifies that:
- A large telemetry JSON split into chunks is correctly reassembled.
- A single-chunk message works (same as direct path).
- Chunks arriving out of order are reassembled correctly.
- Incomplete chunk sequences are discarded on timeout.
- Oversized assembles are rejected.
- Corrupt chunk headers are discarded without crash.
- Chunking mode routes correctly (chunk header detected).
- Direct mode (chunking_enabled=False) handles chunks as raw JSON gracefully.
"""

import logging
import zlib

from aqi_bridge.ble import BLEDroneClient

logging.basicConfig(level=logging.DEBUG)


def make_chunk_frames(json_str: str, chunk_size: int, seq_id: int) -> list[bytes]:
    """
    Simulate Arduino chunked framing.
    Now signs the json_str with CRC32 before chunking to satisfy protocol hardening.
    """
    payload = json_str.encode("utf-8")
    # Sign the payload
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    signed_payload = payload + f"|{crc:08x}".encode("ascii")
    
    chunks = [signed_payload[i:i + chunk_size] for i in range(0, len(signed_payload), chunk_size)]
    total = len(chunks)
    frames = []
    for idx, chunk in enumerate(chunks):
        header = f"{seq_id}|{total}|{idx}|".encode("utf-8")
        frames.append(header + chunk + b"\n")
    return frames

def _with_crc(payload: bytes) -> bytes:
    """Helper for manual frames."""
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    return payload + f"|{crc:08x}".encode("ascii")


def notify(client: BLEDroneClient, frame: bytes) -> None:
    client._on_telemetry_notification(0, bytearray(frame))


def test_chunk_single_message():
    """A 200-byte JSON split across 10x20-byte chunks reassembles correctly."""
    print("\n--- Chunk Test 1: Multi-chunk Assembly ---")
    payload = '{"status":"chunked","aqi":42,"tvoc":500,"eco2":1200,"temperature_c":23.5,"humidity_pct":55.2}'
    client = BLEDroneClient()
    client.chunking_enabled = True
    client.latest_telemetry = None

    frames = make_chunk_frames(payload, chunk_size=20, seq_id=1)
    print(f"  Splitting {len(payload)} bytes into {len(frames)} chunks of ≤20 bytes")
    for frame in frames:
        notify(client, frame)

    assert client.latest_telemetry is not None, "Message was not reassembled"
    assert client.latest_telemetry.aqi == 42
    assert client.latest_telemetry.tvoc == 500
    print("  SUCCESS")


def test_chunk_single_chunk():
    """A tiny JSON sent as a single chunk (total=1) works correctly."""
    print("\n--- Chunk Test 2: Single-chunk Message ---")
    client = BLEDroneClient()
    client.chunking_enabled = True
    client.latest_telemetry = None

    payload_p2 = _with_crc(b'{"status":"single"}')
    frame = b'0|1|0|' + payload_p2 + b'\n'
    notify(client, frame)
    assert client.latest_telemetry is not None
    assert client.latest_telemetry.status == "single"
    print("  SUCCESS")


def test_chunk_out_of_order():
    """Chunks arriving in reverse order are reassembled correctly."""
    print("\n--- Chunk Test 3: Out-of-Order Chunks ---")
    payload = '{"status":"ooo","aqi":99}'
    client = BLEDroneClient()
    client.chunking_enabled = True
    client.latest_telemetry = None

    frames = make_chunk_frames(payload, chunk_size=10, seq_id=2)
    for frame in reversed(frames):
        notify(client, frame)

    assert client.latest_telemetry is not None, "Out-of-order assembly failed"
    assert client.latest_telemetry.aqi == 99
    print("  SUCCESS")


def test_chunk_incomplete_sequence_expired():
    """An incomplete sequence is discarded after timeout, not held forever."""
    print("\n--- Chunk Test 4: Incomplete Sequence Timeout GC ---")
    client = BLEDroneClient()
    client.chunking_enabled = True
    client.latest_telemetry = None

    # Send only chunk 0 of 3
    notify(client, b'7|3|0|{"part":"a"\n')

    assert 7 in client._assemblies, "Assembly record not created"

    # Manually age the record
    client._assemblies[7].created_at -= 5.0  # 5 seconds ago

    # Next notification triggers GC
    ok_payload = _with_crc(b'{"status":"after_gc"}')
    notify(client, b'99|1|0|' + ok_payload + b'\n')

    assert 7 not in client._assemblies, "Expired assembly not cleaned up"
    assert client.latest_telemetry.status == "after_gc"
    print("  SUCCESS")


def test_chunk_oversized_assembly_rejected():
    """If assembled chunks exceed CHUNK_MAX_MESSAGE_SIZE, the sequence is rejected."""
    print("\n--- Chunk Test 5: Oversized Assembly Rejected ---")
    from aqi_bridge.config import CHUNK_MAX_MESSAGE_SIZE
    client = BLEDroneClient()
    client.chunking_enabled = True
    client.latest_telemetry = None

    # Send chunks totalling > 2KB
    big_data = "X" * (CHUNK_MAX_MESSAGE_SIZE + 100)
    frames = make_chunk_frames(big_data, chunk_size=200, seq_id=10)
    for frame in frames:
        notify(client, frame)

    assert client.latest_telemetry is None, "Oversized assembly should have been rejected"
    assert 10 not in client._assemblies, "Oversized sequence should have been purged"
    print("  SUCCESS")


def test_chunk_corrupt_header_discarded():
    """Frames with unparseable chunk headers are silently discarded."""
    print("\n--- Chunk Test 6: Corrupt Header Discarded ---")
    client = BLEDroneClient()
    client.chunking_enabled = True
    client.latest_telemetry = None

    # Wrong number of pipe segments
    notify(client, b'bad|header\n')      # only 2 parts
    notify(client, b'x|y|z|{\n')        # non-integer fields
    ok_p = _with_crc(b'{"status":"ok_after_corrupt"}')
    notify(client, b'99|1|0|' + ok_p + b'\n')

    assert client.latest_telemetry is not None
    assert client.latest_telemetry.status == "ok_after_corrupt"
    print("  SUCCESS")


def test_mtu_json_bytes_limit_enforced():
    """Verify that a fully assembled JSON exceeding MAX_TELEMETRY_JSON_BYTES is dropped."""
    print("\n--- Chunk Test 7: MTU MAX_TELEMETRY_JSON_BYTES Enforcement ---")
    from aqi_bridge.config import MAX_TELEMETRY_JSON_BYTES
    client = BLEDroneClient()
    client.chunking_enabled = True
    client.latest_telemetry = None

    # Create a giant valid JSON payload that exceeds the limit
    base = '{"status":"toolarge", "pad":"X'
    oversized = base + 'X' * (MAX_TELEMETRY_JSON_BYTES + 10) + '"}'
    
    # Needs to be small chunks so it doesn't trigger the CHUNK_MAX_MESSAGE_SIZE limit
    frames = make_chunk_frames(oversized, chunk_size=200, seq_id=11)
    for frame in frames:
        notify(client, frame)

    assert client.latest_telemetry is None, "Telemetry exceeding MAX_TELEMETRY_JSON_BYTES should be dropped!"
    assert 11 not in client._assemblies, "Assembly should be purged after processing."
    print("  SUCCESS")


def test_mtu_startup_defaults():
    """Verify that a newly created client starts with safe conservative defaults."""
    print("\n--- Chunk Test 8: Safe MTU Defaults on Init ---")
    client = BLEDroneClient()
    assert client.negotiated_mtu == 23, f"Expected 23, got {client.negotiated_mtu}"
    assert client.usable_payload == 20, f"Expected 20, got {client.usable_payload}"
    assert client.chunking_enabled is True, "Chunking should default to enabled"
    print("  SUCCESS")


if __name__ == "__main__":
    try:
        test_chunk_single_message()
        test_chunk_single_chunk()
        test_chunk_out_of_order()
        test_chunk_incomplete_sequence_expired()
        test_chunk_oversized_assembly_rejected()
        test_chunk_corrupt_header_discarded()
        test_mtu_json_bytes_limit_enforced()
        test_mtu_startup_defaults()
        print("\nALL CHUNK/MTU TESTS PASSED")
    except AssertionError:
        import traceback
        traceback.print_exc()
        raise SystemExit(1) from None
