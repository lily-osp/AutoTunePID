# AutoTunePID — Project Roadmap & TODO

This document tracks the evolution of `AutoTunePID` towards becoming a robust, high-performance, and safety-aware control library for general microcontrollers.

## 🎯 Design Principles
- **Predictable Behavior:** Ensure control logic executes consistently across different MCU architectures.
- **Explicit Runtime Semantics:** Make state changes and operational limits clear to the user.
- **Zero Dynamic Allocation:** Absolute ban on `malloc`, `new`, and heap usage for stability.
- **Embedded-First:** Optimized for constrained MCUs (AVR, ESP, STM32) without demanding RTOS architectures initially.

---

## 🔥 P0 — Core Stability & Control Refinement

### Control Logic Upgrades
- [x] **Derivative-on-Measurement:** Implement $D = -K_d \cdot \frac{d(Measurement)}{dt}$ to eliminate "derivative kick" when the setpoint is changed abruptly.
- [x] **Output Slew-Rate Limiting:** Protect actuators (motors, valves) by constraining the maximum rate of change ($du/dt$) of the output.
- [ ] **Advanced Anti-Windup:** Implement configurable modes:
  - Clamping (conditional integration) to replace simple limiters.
  - Back-calculation for smoother recovery from saturation.

### Autotuner Reliability
- [x] **Relay-Feedback Model with Hysteresis:** Rebuild the tuner around a robust relay-feedback mechanism that includes noise-rejecting hysteresis.
- [ ] **Tuning State Machine:** Implement an explicit state machine for the auto-tuning process:
  - `IDLE` → `EXCITATION` → `OSCILLATION_DETECT` → `CALCULATING` → `COMPLETED` / `FAULT`.
- [ ] **Plant-Class Presets:** Provide heuristics or starting parameters for specific plant types (Thermal, High-Inertia Motor, Fluid Dynamics).

### Numerical Safety
- [ ] **Comprehensive Safety Guards:** Integrated detection for `NaN`, `Inf`, and arithmetic overflows throughout the PID math.
  - *Interface:* `PIDFault get_last_fault() noexcept;`

---

## 🏗️ P1 — Advanced Features & Observability

### Flexibility
- [ ] **Explicit Delta-Time API (Overload):** Add `pid.update(input, dt_us)` alongside the standard `update(input)` to allow advanced users to inject deterministic timing (e.g., from an ISR) while keeping the standard API simple.
- [ ] **Feedforward Support:** Add a direct feedforward injection point: `output = pid_calc + feedforward;`.
- [ ] **Gain Scheduling:** Enable runtime switching of PID constants based on operating regions.

### Diagnostics
- [ ] **Telemetry Interface:** Expose internal terms ($P, I, D$) and saturation states via getter methods for user-defined logging (decoupled from `Serial`).
- [ ] **Trace-Level Instrumentation:** Add optional `ATPID_ENABLE_TRACE` macros for deep debugging.

---

## 🛡️ P2 — Architecture & Engineering Excellence

### Modular Decoupling
- [ ] **Platform-Agnostic Core:** Decouple from framework-specific headers (e.g., `Arduino.h`) to support bare-metal STM32 or other environments.
- [ ] **Strict Directory Separation:** Organize into `core/` (Logic), `tuning/` (Optional), `filters/` (Pluggable), `platform/` (Hardware Abstraction).
- [ ] **Strong Type Safety:** Replace remaining magic integers with scoped `enum class` configurations.

### Verification
- [ ] **Simulated Plant Regression:** CI suite testing against modeled first-order and second-order systems.
- [ ] **Static Analysis:** Integrate `clang-tidy` (MISRA profile) and `cppcheck` into the workflow.

---

## 📝 Documentation Requirements
- [ ] **Timing Guide:** Best practices for loop frequency and jitter mitigation on standard MCUs.
- [ ] **Failure Catalog:** Visual guide to troubleshooting issues like integral runaway or noise amplification.
