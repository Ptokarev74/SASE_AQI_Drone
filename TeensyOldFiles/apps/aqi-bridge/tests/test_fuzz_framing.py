"""
Property-based fuzzing for the AQI Bridge BLE framing and reassembly layer.

Validates the robust invariant: "The bridge never crashes on garbage data."
It does so by feeding massive quantities of randomized, malformed, and truncated
bytes into the `BLEDroneClient._on_telemetry_notification` parser.

Invariants Asserted:
1. No unhandled exceptions ever escape the parsing sequence.
2. The internal byte buffer (`_buffer`) never grows unbounded (MAX_TELEMETRY_BUFFER_SIZE).
3. The chunk assembly dictionaries (`_assemblies`) do not leak memory infinitely.
"""

import zlib

from aqi_bridge.ble import MAX_TELEMETRY_BUFFER_SIZE, BLEDroneClient
from hypothesis import given, settings
from hypothesis import strategies as st

# Deterministic limits for CI execution
settings.register_profile("ci", max_examples=500, deadline=None)
settings.load_profile("ci")

@given(st.binary(min_size=1, max_size=MAX_TELEMETRY_BUFFER_SIZE * 2))
def test_fuzz_unstructured_noise(fuzzed_bytes: bytes):
    """
    Feed completely unstructured noise (random bytes) directly into the notification handler.
    Asserts the pipeline never raises an unhandled exception or panics.
    """
    ble = BLEDroneClient()
    
    # Send fuzzed bytes
    ble._on_telemetry_notification(1, bytearray(fuzzed_bytes))
    
    # Assert buffer constraints
    assert len(ble._buffer) <= MAX_TELEMETRY_BUFFER_SIZE


@given(
    seq_id=st.integers(min_value=-10, max_value=300),
    total=st.integers(min_value=-10, max_value=300),
    idx=st.integers(min_value=-10, max_value=300),
    corruption=st.sampled_from([b"", b"|", b"\n", b"\\x00", b"random"]),
    data=st.binary()
)
def test_fuzz_malformed_chunk_headers(seq_id, total, idx, corruption, data):
    """
    Fuzz the highly-structured chunk sequence logic (seq|total|idx|data).
    Ensures that adversarial chunk definitions (e.g. idx > total, total < 1)
    or corrupted payload structures do not cause out-of-bounds array access 
    or memory leaks.
    """
    ble = BLEDroneClient()
    ble.chunking_enabled = True
    
    # Construct a malformed chunk string, delimited by newline to force parser branch
    header = f"{seq_id}|{total}|{idx}|".encode("utf-8")
    malformed_frame = header + corruption + data + b"\n"
    
    ble._on_telemetry_notification(1, bytearray(malformed_frame))
    
    # Assert that if the structure is invalid, it doesn't leak into permanent assembly memory
    if total < 1 or total > 255 or idx < 0 or idx >= total:
        assert seq_id not in ble._assemblies


@given(
    payload=st.text(alphabet=st.characters(blacklist_categories=('Cs',)), min_size=1, max_size=2000)
)
def test_fuzz_json_payload_resilience(payload: str):
    """
    Feed validly framed but internally corrupted (or malicious) text payloads.
    Asserts JSON decoding and Pydantic validation handles arbitrary malformed structures
    without crashing the service.
    """
    ble = BLEDroneClient()
    
    # Frame it perfectly (with valid CRC) so it reaches the JSON parser
    payload_bytes = payload.encode("utf-8")
    crc = zlib.crc32(payload_bytes) & 0xFFFFFFFF
    frame = payload_bytes + f"|{crc:08x}\n".encode("ascii")
    ble._on_telemetry_notification(1, bytearray(frame))
    
    # Assert the parser survived. It should either parse successfully or quietly discard.
    assert len(ble._buffer) == 0

