# Engineering Tradeoffs: Interview Guide

When asked about the AQI Drone Bridge in an engineering context, frame the system as a set of constraint-driven decisions rather than a feature list. Use the **Constraint → Failure Mode → Tradeoff → Result** structure.

Here are 5 interview-ready explanations for the core architecture.

---

## 1. The Command Pipeline (Queue Size = 4)

**The Tradeoff:** Bounded Latency over Command Completeness
**The Setup:** "In this system, a React frontend sends flight commands over WebSocket to a Python bridge, which forwards them over BLE to the drone."
**The Explanation:**
> "The constraint is that the phone generates WebSocket messages much faster than Bluetooth can transmit them. If I used a standard unbounded queue, the queue would back up under rapid stick movements. 
> 
> The failure mode is unbounded latency—the drone would execute a command from seconds ago instead of the pilot's current stick input, which guarantees a crash. 
>
> I traded command completeness for bounded latency by strictly limiting the queue size to 4 and implementing a 'drop-oldest' policy. At a 30Hz target rate, this caps our worst-case command latency at around 130 milliseconds. In an active control system, bounded latency is critical; a stale command is a dangerous command."

---

## 2. Bluetooth Concurrency (Single-Writer Pattern)

**The Tradeoff:** Safety over Parallel Throughput
**The Setup:** "The Python bridge handles multiple async tasks: reading telemetry, accepting WebSocket commands, and running a deadman safety timer."
**The Explanation:**
> "The underlying constraint is that OS Bluetooth stacks and Arduino hardware fall over if you attempt concurrent GATT write operations. 
>
> Originally, if the pilot sent a command at the exact millisecond the deadman timer fired, two coroutines would write to the BLE socket simultaneously. The failure mode was interleaved byte streams causing unpredictable hardware lockups mid-flight.
>
> I traded parallel throughput for absolute safety by forcing all writes through a single bottleneck queue consumer. Emergency stop commands wait in the same queue behind a maximum of 4 normal commands rather than risking a hardware lockup. Predictable safety is better than theoretical speed."

---

## 3. The Telemetry Framing Protocol (NDJSON)

**The Tradeoff:** Structural Simplicity over Binary Compactness
**The Setup:** "The drone beams down a 300-byte telemetry payload containing GPS, gyro, and air quality data, which exceeds standard BLE MTU limits."
**The Explanation:**
> "The constraint is that Bluetooth breaks large payloads into unpredictable 20-250 byte fragments. 
>
> If I relied on the OS packet boundaries, the Python client would receive partial JSON strings. The failure mode is framing desynchronization—losing track of where one payload ends and the next begins.
>
> I traded binary compactness for structural simplicity by enforcing Newline-Delimited JSON tracking. The Arduino terminates every payload with a `\n` character. The Python bridging layer simply buffers bytes and splits on newlines. We use 1 extra byte of bandwidth per message, but we are completely immune to fragmentation bugs and our parsing logic is radically simpler."

---

## 4. Shared State Memory Safety (Immutability)

**The Tradeoff:** Memory Allocation over Lock Contention
**The Setup:** "The bridge receives telemetry from the drone asynchronously while simultaneously fanning it out to mobile viewers at 30Hz."
**The Explanation:**
> "The constraint is that serializing complex JSON payloads is slow, and we cannot hold a mutex lock during that process without stalling the BLE reader.
>
> If we serialized a shared dictionary outside the lock while the BLE loop updated it, the failure mode is a runtime concurrency crash or silently transmitting a hybrid of old and new data fields.
>
> I traded memory allocation overhead for lock-free concurrency by enforcing immutable replacement. We use frozen Pydantic models. The background loop allocates a brand new object on every reading. The broadcast loop locks for less than 1 microsecond just to grab a reference copy, and serializes safely outside the lock. The result is zero lock contention with guaranteed JSON consistency."

---

## 5. The Reliability Strategy (Fail-Fast Policy)

**The Tradeoff:** Deterministic Failure Containment over Process Resilience
**The Setup:** "The Python bridge runs unattended on a companion computer or ground station."
**The Explanation:**
> "The constraint is that a Python application running multiple async loops can silently degrade—for instance, the WebSocket server crashes but the BLE loop keeps running.
>
> The failure mode is a 'zombie' bridge that appears alive but cannot actually convey commands to the drone, which is functionally equivalent to a system failure but much harder to monitor.
>
> I traded internal resilience for deterministic failure containment by implementing a strict Fail-Fast policy. `main.py` supervises all tasks; if any task throws an unhandled exception, the entire process intentionally exits with status 1. We delegate absolute recovery to an external OS supervisor like `systemd`. We want clear, actionable failures, not hidden degradation."
