# AutoTunePID — Project Roadmap & TODO

This document tracks the evolution of `AutoTunePID` from a hobbyist-level library to an industrial-grade, deterministic, and safety-critical embedded control system.

## Design Principles
- **Deterministic Behavior:** Predictable execution time and control logic.
- **Explicit Runtime Semantics:** No hidden timing or state dependencies.
- **Zero Dynamic Allocation:** Absolute ban on `malloc`, `new`, and heap usage.
- **Platform-Agnostic Core:** Decoupled from framework-specific headers (e.g., `Arduino.h`).
- **Embedded-First:** Optimized for constrained MCUs and RTOS environments.

---

## P0 — Core Stability & Control Correctness

### Deterministic Runtime
- [ ] **Explicit Delta-Time API:** Transition from internal `millis()` to dependency-injected intervals.
  - *Target:* `pid.update(input, dt_us);`
  - *Goal:* Ensure ISR and RTOS safety; remove framework coupling.
- [ ] **Resource Characterization:** Document worst-case execution time (WCET), stack depth, and flash footprint.
- [ ] **Compile-Time Validation:** Use `static_assert` to validate gain ranges and configuration parameters.

### Control Logic Refinement
- [ ] **Derivative-on-Measurement:** Implement $D = -K_d \cdot \frac{d(Measurement)}{dt}$ to eliminate "derivative kick" during setpoint changes.
- [ ] **Advanced Anti-Windup:** Implement configurable modes:
  - Clamping (conditional integration).
  - Back-calculation for smoother saturation recovery.
- [ ] **Numerical Safety Guards:** Integrated detection for `NaN`, `Inf`, and arithmetic overflows.
  - *Interface:* `PIDFault get_last_fault() noexcept;`
- [ ] **Output Slew-Rate Limiting:** Protect actuators by constraining the maximum rate of change ($du/dt$).

### Autotuner Reliability
- [ ] **Relay-Feedback Model:** Rebuild the tuner around a robust relay-feedback mechanism with hysteresis.
- [ ] **Tuning State Machine:** Implement an explicit state machine:
  - `IDLE` → `EXCITATION` → `OSCILLATION_DETECT` → `CALCULATING` → `COMPLETED` / `FAULT`.
- [ ] **Plant-Class Presets:** Provide heuristics for specific plant types (Thermal, High-Inertia Motor, Fluid Dynamics).

---

## P1 — Architecture & Observability

### Modular Decoupling
- [ ] **Strict Directory Separation:**
  - `core/` (Logic), `tuning/` (Optional), `filters/` (Pluggable), `platform/` (Hardware Abstraction).
- [ ] **Strong Type Safety:** Replace remaining magic integers with scoped `enum class` configurations.
- [ ] **Template-Driven Configuration:** Use policy-based design for features like filtering to avoid virtual dispatch overhead.

### Diagnostics
- [ ] **Telemetry Interface:** Expose internal terms ($P, I, D$) and saturation states without coupling to `Serial`.
- [ ] **Trace-Level Instrumentation:** Add `ATPID_ENABLE_TRACE` macros for deep debugging.

---

## P2 — Engineering Excellence

### Signal Integrity
- [ ] **Adaptive Filtering:** Replace static EMA with $dt$-aware low-pass filters for derivative and input signals.
- [ ] **Fixed-Point Support:** Provide a template specialization for fixed-point arithmetic (e.g., Q16.16) for MCUs lacking an FPU.

### Verification
- [ ] **Simulated Plant Regression:** CI suite testing against modeled first-order and second-order systems.
- [ ] **Static Analysis:** Integrate `clang-tidy` (MISRA profile) and `cppcheck` into the workflow.

---

## P3 — Advanced Features
- [ ] **Feedforward Support:** Add a direct feedforward injection point: `output = pid_calc + feedforward;`.
- [ ] **Gain Scheduling:** Enable runtime switching of PID constants based on operating regions.
- [ ] **Cascaded Controller Support:** Simplify the wiring of nested loops.

---

## Documentation Requirements
- [ ] **Architecture Manifesto:** Explain the shift to deterministic engineering.
- [ ] **Timing Guide:** Best practices for loop frequency and jitter mitigation.
- [ ] **Failure Catalog:** Visual guide to troubleshooting issues like integral runaway.
