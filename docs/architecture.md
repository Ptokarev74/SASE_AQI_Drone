# SASE AQI Drone: Deadman Safety Contract

This document formalizes the rigorous, verifiable safety guarantees inherent to the AQI Bridge system regarding telemetry loss and pilot disconnection.

Prior assumed "failsafe" models have been entirely replaced by explicitly measured and statistically qualified system contracts.

## The Deadman Precedent
In the event that the system receives no new control input from a pilot over the WebSocket (`DEADMAN_TIMEOUT_MS = 500.0` ms), or explicitly loses the BLE connection to the drone, the bridge instantly initiates the Deadman Safety Contract.

## The Contract Invariants

1. **Explicit Autonomous Action**: The system injects a specialized, pre-built `FAILSAFE_COMMAND` into the broadcast pipeline instead of reusing a neutral pilot command. It zeroes out all velocity vectors, sets `arm: False`, and marks `is_failsafe: True`. 

2. **Strict Injection Budgets**: From the moment the 500ms silence threshold is crossed, the bridge is strictly required to emit the `FAILSAFE_COMMAND` within `FAILSAFE_EMIT_BUDGET_MS` (50.0 ms).

3. **Control Inhibition**: Upon triggering a failsafe emission, the Python bridge enters a **Control Inhibited** state. Even if the WebSocket connection recovers and the pilot is pegging a joystick full forward, the bridge will silently drop all arriving telemetry. This is specifically to prevent violent "rubber-banding" phenomena where a drone, gracefully descending in a failsafe state, suddenly lurches laterally at maximum velocity due to stale client-side input buffering.

4. **Explicit Re-Arm Requirement**: Control is unconditionally locked out until the pilot issues a control command with an explicit `arm: True` signature. Only then is the `control_inhibited` state cleared, and standard telemetry routing resumes.

## Certification and Validation
These components are not "best effort" — they are systematically enforced boundaries. 

* The `test_deadman_contract.py` suite explicitly asserts the bridge's adherence to the emission budgets and the inhibition state-machine locks. 
* Total system safety (Bridge processing + BLE broadcast + Arduino Motor-Stop execution) goes through **Hardware-in-the-Loop (HIL)** qualification via `apps/aqi-bridge/tests/test_hil_failsafe_qualify.py`. This certification physically monitors the specific microsecond timestamp the Arduino shuts down its PWM generators through a USB side-channel to unequivocally prove the transition occurs within safety boundaries.

## Failure Mode Inventory & Coverage Matrix
This system guarantees verifiable behaviors under adversarial or pathological degradation scenarios. Every identified major edge-case is classified against strict verification standards.

| Failure Mode | Trigger Condition | Expected Invariant | Detection & Mitigation | Validation Stage |
| --- | --- | --- | --- | --- |
| **BLE Disconnect Storm** | Extreme RF interference or link-flap | Single reconnect recovery | Graceful link tear-down `_safe_disconnect()` to force OS BLE clean slate. | **CI**: Fuzzed buffer. **HIL**: Disconnect load. |
| **BLE Backpressure** | TX characteristics blocked | Bridge never CPU locks | Single asynchronous enqueue path limits drops. | **CI/Perf**: Write timeout caps (`test_ble_single_writer.py`) |
| **Command Staleness** | Latency buffers command | Drone receives intent `< MAX_COMMAND_AGE_MS` (300.0 ms) | Freshness is enforced by both bounded queue depth and explicit MAX_COMMAND_AGE_MS rejection at dequeue and write time. | **CI**: Asserted manually in `test_freshness.py` and `test_failure_modes.py` (see `apps/aqi-bridge/tests/`) |
| **Long-run Memory Leak** | Indefinite continuous operation | Usage remains `< 150MB` RSS | Internal queues are bounded; references are atomic. | **Perf**: Verified continuously with fuzzed streaming via `psutil`. |
| **Pathological JSON Payload** | Malicious injection or data corruption (e.g. `NaN`) | Safe numeric execution limits only | Pydantic strict bounding (`allow_inf_nan=False`, explicit bounds) rejects at decode before logic hits. | **CI**: Bounded testing in `test_failure_modes.py` (see `apps/aqi-bridge/tests/`) |
| **Multi-client Malicious Sender** | Websocket spam attacks | Queue remains responsive to safe clients | Total client limit `MAX_WS_CLIENTS=10` limits attack surface, drop-oldest ensures buffer clears. | **CI**: Bounded queue length tests. |
| **Clock/Time Anomaly** | System time leaps (NTP adjustments) | Budgets do not trigger falsely | Exclusively using `time.monotonic()` for all temporal budgets and failsafe tracking natively avoids NTP glitches. | **Architecture Check** |
| **Supervisor Crash Loop** | Bad firmware code causes panic | CPU does not lock 100% | Bounded Exponential backoff (1s -> 60s) injected inside the main wrapper thread delays restart thrashing. | **CI**: Boot tested mathematically in `test_failure_modes.py` (see `apps/aqi-bridge/tests/`). |
| **Auth Abuse (DDoS/Spam)** | Flooding connection handshakes | Server process stands | Explicit WS rejection without full session generation. Optional telemetry-only mode isolates logic constraints. | **Not explicitly rate-limited on IP** (Out-of-Scope: Offloaded to WAF proxy layer in prod). |
