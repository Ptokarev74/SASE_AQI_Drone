# Performance Qualification

## 1. Scope of Qualification
The Python Bridge serves as the reliable middleman between WebSockets and Bluetooth Low Energy (BLE). While continuous integration (CI) tests validate functional correctness and safety invariants (e.g., dropping stale frames), CI environments run on shared, non-deterministic virtual machines.

**CI is not a valid benchmark for real-time determinism.**

To assert real-time behavior, the bridge must undergo statistical Performance Qualification on dedicated, isolated hardware. This document outlines the expected execution environment, measurement methodology, and explicit numeric budgets required to certify the bridge for flight constraints.

---

## 2. Hardware & Environment Specifications
Performance Qualification runs must be executed under the following strict conditions. Any deviation invalidates the real-time assertions.

1. **Hardware Node**: A dedicated, bare-metal machine (e.g., Raspberry Pi 4/5 or dedicated Linux host).
2. **CPU Governor**: Pinned CPU frequencies. No frequency scaling (e.g., `cpufreq-set -g performance`).
3. **Networking**: No background network payloads; isolated loopback for the orchestrator.
4. **Bluetooth**: A physically stable BLE adapter with no competing pairings or ambient sweeps during the test window.

---

## 3. Numeric Latency Budgets
Every command passing through the bridge is instrumented with `time.monotonic()` for precision timing. The performance profile evaluates the **End-to-End Delivery Latency** defined as:

`End-to-End Latency = ts_dispatched - ts_received`

The bridge must assert the following latency budgets across a prolonged stress-test execution phase:

* **Target Mean Latency**: < 40ms
* **P95 Latency**: < 80ms
* **P99 Latency (MAX_E2E_LATENCY_MS)**: 120ms
* **Absolute Worst Case**: < 150ms

### Jitter and Consistency
Real-time control demands consistency. The variance (jitter) between back-to-back command latency outputs must not exceed **20ms**. Spikes representing garbage collection or orchestration pauses exceeding 150ms constitute a failed qualification limit.

### Environment-Specific Jitter Considerations

#### Windows Terminal Overhead
On Windows, writing to the standard console is a synchronous and relatively slow operation that can introduce significant jitter into the real-time command pipeline. To mitigate this:

1. **Asynchronous Logging**: The bridge implements a background logging thread (`QueueHandler`/`QueueListener`) so that `logger` calls in the command pipeline return instantly without waiting for the terminal to refresh.
2. **Quiet Execution**: For the most deterministic results on Windows, run qualification with `LOG_LEVEL=ERROR` or redirect the log output to `NUL` to minimize OS-level terminal scheduling overhead.

---

## 4. Performance Qualification Methodology
The standalone qualification script (`apps/aqi-bridge/tests/test_perf_qualify.py`) systematically simulates worst-case environments.

**Test Matrix:**
1. **Maximum WebSocket Load**: 10 simultaneous connected clients broadcasting telemetry.
2. **Burst Command Floods**: Input bursts violating the dequeuing bounds precisely to ensure `drop_oldest` is executed in a non-blocking sequence.
3. **Execution Cycles**: Minimum of 30 isolated measurement runs.

### Reporting
The qualification stage requires numeric proof to be traced to physical artifacts. The script returns an exit code of `1` alongside its statistical bounds if the *Worst Case* limit breaches the 150ms margin bounding safety limits.
