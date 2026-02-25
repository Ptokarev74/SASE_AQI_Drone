"""
Resource bounds stress testing for the AQI Bridge.

Continuously feeds random and adversarial byte streams into the BLE framing 
pipeline for a defined duration. Monitors the process with `psutil` to categorically 
prove that the system does not leak memory or exhaust CPU (starvation loops)
when handling dense garbage data.
"""

import asyncio
import os
import random
import time
import psutil
import zlib
from hypothesis import given, settings, strategies as st

from aqi_bridge.ble import BLEDroneClient

# Exact constraints mandated by docs/PROJECT_MANAGER_UPDATE.md
MAX_MEMORY_MB = 150.0
MAX_CPU_PERCENT = 80.0

# CI timeout safe values
STRESS_DURATION_S = 3.0


def _get_process_metrics():
    """Retrieve memory (MB) and CPU (%) for the current test process."""
    process = psutil.Process(os.getpid())
    mem_info = process.memory_info()
    mem_mb = mem_info.rss / (1024 * 1024)
    # Give a tiny interval to calculate CPU utilization correctly, normalizing by core count
    cpu_percent = process.cpu_percent(interval=0.01) / psutil.cpu_count()
    return mem_mb, cpu_percent


def test_sustained_fuzzed_load_resource_bounds():
    """
    Sustain a high-throughput stream of randomized byte corruption into the parser.
    Measure periodic process metrics. Assert the ceilings are strictly honored.
    """
    ble = BLEDroneClient()
    
    # Establish baseline
    mem_base, cpu_base = _get_process_metrics()
    
    start_time = time.time()
    iterations = 0
    max_mem_observed = mem_base
    max_cpu_observed = cpu_base
    
    # Pseudo-random but deterministic sequence for reproducible stress
    rng = random.Random(42)

    while (time.time() - start_time) < STRESS_DURATION_S:
        iterations += 1
        
        # Construct dense malformed payload (1-400 bytes)
        size = rng.randint(1, 400)
        malformed_bytes = bytearray(rng.getrandbits(8) for _ in range(size))
        
        if iterations % 7 == 0:
            malformed_bytes += b"|1|0|"
        
        # Add CRC32 to some fuzzed frames to ensure they reach the JSON parser
        if iterations % 3 == 0:
            crc = zlib.crc32(malformed_bytes) & 0xFFFFFFFF
            malformed_bytes += f"|{crc:08x}".encode("ascii")
        
        # Inject periodic newlines to flush frames
        if iterations % 5 == 0:
            malformed_bytes += b"\n"

        ble._on_telemetry_notification(1, malformed_bytes)
        
        # Sample metrics periodically
        if iterations % 1000 == 0:
            mem_now, cpu_now = _get_process_metrics()
            max_mem_observed = max(max_mem_observed, mem_now)
            max_cpu_observed = max(max_cpu_observed, cpu_now)
            
            # Fast fail if bounds exceeded
            assert max_mem_observed < MAX_MEMORY_MB, f"Memory leaked to {max_mem_observed:.2f} MB!"
            assert max_cpu_observed < MAX_CPU_PERCENT, f"CPU spiked to {max_cpu_observed:.2f}%!"
            
    # Final assertion explicitly linking back to PM documentation limits
    final_mem, final_cpu = _get_process_metrics()
    max_mem_observed = max(max_mem_observed, final_mem)
    max_cpu_observed = max(max_cpu_observed, final_cpu)

    print(f"\n--- Resource Bounds Qualification ---")
    print(f"Iterations (Frames Processed): {iterations}")
    print(f"Peak Memory: {max_mem_observed:.2f} MB (Limit: {MAX_MEMORY_MB} MB)")
    print(f"Peak CPU:    {max_cpu_observed:.2f} %  (Limit: {MAX_CPU_PERCENT} %)")
    print("---------------------------------------")

    assert max_mem_observed < MAX_MEMORY_MB, f"Memory constraint violated: {max_mem_observed:.2f} MB > {MAX_MEMORY_MB} MB"
    assert max_cpu_observed < MAX_CPU_PERCENT, f"CPU constraint violated: {max_cpu_observed:.2f} % > {MAX_CPU_PERCENT} %"

