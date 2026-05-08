# TODO: AUTOSAR C++14 Compliance Roadmap

This document outlines the prioritized tasks required to bring the `AutoTunePID` library into full compliance with the AUTOSAR C++14 standard for safety-critical systems.

## Priority 0: Core Foundation (Critical)
*Goal: Establish the structural and type-safety baseline.*

- [x] **[P0-1] Namespace Encapsulation (Rule A7-2-1):**
    - Wrap the library in `namespace atp {}`.
    - Provide `using` declarations or a compatibility header for backward compatibility (optional).
- [x] **[P0-2] Fixed-Width Integers (Rule A0-1-1):**
    - Include `<stdint.h>`.
    - Replace all `int` with `int32_t`.
    - Replace all `unsigned long` with `uint32_t`.
- [x] **[P0-3] Explicit Floating-Point Literals (Rule A0-1-6):**
    - Append `f` suffix to all floating-point literals (e.g., `0.5` -> `0.5f`) to prevent implicit `double` promotion.

## Priority 1: Interface & Safety (High)
*Goal: Prevent common C++ pitfalls and ensure strict interface contracts.*

- [x] **[P1-1] Explicit Constructor (Rule A12-1-1):**
    - Mark the `AutoTunePID` constructor as `explicit` to prevent unintended implicit type conversions.
- [x] **[P1-2] Const Correctness (Rule A7-1-1):**
    - Audit all methods and mark non-mutating ones (e.g., getters, `computeFilteredValue`) as `const`.
    - Pass complex types (if any) by `const` reference.
- [x] **[P1-3] Explicit Type Conversions (Rule A5-2-1):**
    - Replace all implicit conversions and C-style casts with `static_cast<T>()`.
    - Pay special attention to `millis()` to `float` conversions and PID term math.
- [x] **[P1-4] Complete Member Initialization (Rule A12-1-6):**
    - Ensure every class member is initialized in the constructor's member initializer list in the order of declaration.

## Priority 2: Documentation & Polish (Standard)
*Goal: Ensure long-term maintainability and compliance with documentation standards.*

- [ ] **[P2-1] Doxygen Documentation (Rule A0-4-1):**
    - Add `@brief`, `@param`, and `@return` tags to all public methods.
    - Document internal state variables using `/**< ... */` format.
- [ ] **[P2-2] Remove Global Dependencies:**
    - If possible, abstract away from `PI` and use a local `constexpr` float to ensure precision consistency.
- [ ] **[P2-3] Logical Cleanup:**
    - Audit `constrain` and `map` usage to ensure they are operating on the correct types without intermediate overflows.

## Verification Tasks
- [ ] Run static analysis audit against the `AUTOSAR C++14` rule set.
- [ ] Compile for AVR, ESP32, and SAMD to verify no architecture-specific warnings.
- [ ] Verify regression tests against all example `.ino` files.
