# Using the AutoTunePID Library

The `AutoTunePID` library is a powerful tool for adaptive PID control in Arduino projects. It features automatic tuning based on methods like **Ziegler-Nichols**, **Cohen-Coon**, **Relay Feedback**, **IMC**, and **Tyreus-Luyben**, as well as manual tuning options. This guide provides detailed examples for each tuning method, including proper pinouts, setpoints, **filtering**, and **anti-windup**.

---

## Table of Contents

1. [Ziegler-Nichols Example with Filtering and Anti-Windup](#ziegler-nichols-example-with-filtering-and-anti-windup)
2. [Cohen-Coon Example with Filtering and Anti-Windup](#cohen-coon-example-with-filtering-and-anti-windup)
3. [Relay Feedback Example with Filtering and Anti-Windup](#relay-feedback-example-with-filtering-and-anti-windup)
4. [IMC Example with Filtering and Anti-Windup](#imc-example-with-filtering-and-anti-windup)
5. [Tyreus-Luyben Example with Filtering and Anti-Windup](#tyreus-luyben-example-with-filtering-and-anti-windup)
6. [Manual Tuning Example with Filtering and Anti-Windup](#manual-tuning-example-with-filtering-and-anti-windup)

---

## Ziegler-Nichols Example with Filtering and Anti-Windup

### Temperature Control System

This example demonstrates how to use the **Ziegler-Nichols** tuning method for a temperature control system with **input and output filtering** and **anti-windup**.

#### Pin Configuration
- **Input Pin**: A0 (Temperature Sensor)
- **Output Pin**: 3 (Heater Control)
- **Setpoint**: 75.0°C

#### Code
```cpp
#include "AutoTunePID.h"

// Initialize PID controller with output range and Ziegler-Nichols method
AutoTunePID tempController(0, 255, TuningMethod::ZieglerNichols);

void setup() {
    tempController.setSetpoint(75.0); // Set target temperature to 75°C
    tempController.enableInputFilter(0.1);  // Enable input filtering with alpha = 0.1
    tempController.enableOutputFilter(0.1); // Enable output filtering with alpha = 0.1
    tempController.enableAntiWindup(true, 0.8); // Enable anti-windup with 80% threshold
}

void loop() {
    float temp = analogRead(A0) * (100.0 / 1023.0); // Read temperature sensor (0-100°C range)
    tempController.update(temp); // Update PID controller
    analogWrite(3, tempController.getOutput()); // Control heater
    delay(100); // Update every 100ms
}
```

---

## Cohen-Coon Example with Filtering and Anti-Windup

### Motor Speed Control System

This example demonstrates how to use the **Cohen-Coon** tuning method for a motor speed control system with **input and output filtering** and **anti-windup**.

#### Pin Configuration
- **Input Pin**: A0 (Encoder or Tachometer)
- **Output Pin**: 5 (Motor PWM Control)
- **Setpoint**: 1500 RPM

#### Code
```cpp
#include "AutoTunePID.h"

// Initialize PID controller with output range and Cohen-Coon method
AutoTunePID motorController(0, 255, TuningMethod::CohenCoon);

void setup() {
    motorController.setSetpoint(1500); // Set target speed to 1500 RPM
    motorController.enableInputFilter(0.2);  // Enable input filtering with alpha = 0.2
    motorController.enableOutputFilter(0.2); // Enable output filtering with alpha = 0.2
    motorController.enableAntiWindup(true, 0.7); // Enable anti-windup with 70% threshold
}

void loop() {
    float rpm = analogRead(A0) * (3000.0 / 1023.0); // Read RPM (0-3000 RPM range)
    motorController.update(rpm); // Update PID controller
    analogWrite(5, motorController.getOutput()); // Control motor speed
    delay(100); // Update every 100ms
}
```

---

## Relay Feedback Example with Filtering and Anti-Windup

### Water Level Control System

This example demonstrates how to use the **Relay Feedback** tuning method for a water level control system with **input and output filtering** and **anti-windup**.

#### Pin Configuration
- **Input Pin**: A0 (Water Level Sensor)
- **Output Pin**: 6 (Valve Control)
- **Setpoint**: 50.0 cm

#### Code
```cpp
#include "AutoTunePID.h"

// Initialize PID controller with output range and Relay Feedback method
AutoTunePID waterController(0, 255, TuningMethod::RelayFeedback);

void setup() {
    waterController.setSetpoint(50.0); // Set target water level to 50 cm
    waterController.enableInputFilter(0.15);  // Enable input filtering with alpha = 0.15
    waterController.enableOutputFilter(0.15); // Enable output filtering with alpha = 0.15
    waterController.enableAntiWindup(true, 0.9); // Enable anti-windup with 90% threshold
}

void loop() {
    float level = analogRead(A0) * (100.0 / 1023.0); // Read water level (0-100 cm range)
    waterController.update(level); // Update PID controller
    analogWrite(6, waterController.getOutput()); // Control valve
    delay(100); // Update every 100ms
}
```

---

## IMC Example with Filtering and Anti-Windup

### Pressure Control System

This example demonstrates how to use the **IMC** tuning method for a pressure control system with **input and output filtering** and **anti-windup**.

#### Pin Configuration
- **Input Pin**: A0 (Pressure Sensor)
- **Output Pin**: 9 (Pump Control)
- **Setpoint**: 100.0 kPa

#### Code
```cpp
#include "AutoTunePID.h"

// Initialize PID controller with output range and IMC method
AutoTunePID pressureController(0, 255, TuningMethod::IMC);

void setup() {
    pressureController.setSetpoint(100.0); // Set target pressure to 100 kPa
    pressureController.enableInputFilter(0.1);  // Enable input filtering with alpha = 0.1
    pressureController.enableOutputFilter(0.1); // Enable output filtering with alpha = 0.1
    pressureController.enableAntiWindup(true, 0.8); // Enable anti-windup with 80% threshold
}

void loop() {
    float pressure = analogRead(A0) * (200.0 / 1023.0); // Read pressure (0-200 kPa range)
    pressureController.update(pressure); // Update PID controller
    analogWrite(9, pressureController.getOutput()); // Control pump
    delay(100); // Update every 100ms
}
```

---

## Tyreus-Luyben Example with Filtering and Anti-Windup

### Chemical Reactor Temperature Control

This example demonstrates how to use the **Tyreus-Luyben** tuning method for a chemical reactor temperature control system with **input and output filtering** and **anti-windup**.

#### Pin Configuration
- **Input Pin**: A0 (Temperature Sensor)
- **Output Pin**: 10 (Heater Control)
- **Setpoint**: 80.0°C

#### Code
```cpp
#include "AutoTunePID.h"

// Initialize PID controller with output range and Tyreus-Luyben method
AutoTunePID reactorController(0, 255, TuningMethod::TyreusLuyben);

void setup() {
    reactorController.setSetpoint(80.0); // Set target temperature to 80°C
    reactorController.enableInputFilter(0.1);  // Enable input filtering with alpha = 0.1
    reactorController.enableOutputFilter(0.1); // Enable output filtering with alpha = 0.1
    reactorController.enableAntiWindup(true, 0.8); // Enable anti-windup with 80% threshold
}

void loop() {
    float temp = analogRead(A0) * (100.0 / 1023.0); // Read temperature (0-100°C range)
    reactorController.update(temp); // Update PID controller
    analogWrite(10, reactorController.getOutput()); // Control heater
    delay(100); // Update every 100ms
}
```

---

## Manual Tuning Example with Filtering and Anti-Windup

### Generic Control System

This example demonstrates how to use **manual tuning** for a generic control system with **input and output filtering** and **anti-windup**.

#### Pin Configuration
- **Input Pin**: A0 (Sensor Input)
- **Output Pin**: 11 (Actuator Control)
- **Setpoint**: 50.0 (Arbitrary Units)

#### Code
```cpp
#include "AutoTunePID.h"

// Initialize PID controller with output range and manual tuning
AutoTunePID manualController(0, 255, TuningMethod::Manual);

void setup() {
    manualController.setSetpoint(50.0); // Set target value to 50
    manualController.setManualGains(1.0, 0.5, 0.1); // Set Kp, Ki, Kd manually
    manualController.enableInputFilter(0.1);  // Enable input filtering with alpha = 0.1
    manualController.enableOutputFilter(0.1); // Enable output filtering with alpha = 0.1
    manualController.enableAntiWindup(true, 0.8); // Enable anti-windup with 80% threshold
}

void loop() {
    float input = analogRead(A0) * (100.0 / 1023.0); // Read sensor input (0-100 range)
    manualController.update(input); // Update PID controller
    analogWrite(11, manualController.getOutput()); // Control actuator
    delay(100); // Update every 100ms
}
```

---

## Summary of Examples with Filtering and Anti-Windup

| Tuning Method       | Example Application          | Input Pin | Output Pin | Setpoint       | Input Filter (α) | Output Filter (α) | Anti-Windup Threshold |
|---------------------|------------------------------|-----------|------------|----------------|------------------|-------------------|------------------------|
| **Ziegler-Nichols** | Temperature Control          | A0        | 3          | 75.0°C         | 0.1              | 0.1               | 80%                    |
| **Cohen-Coon**      | Motor Speed Control          | A0        | 5          | 1500 RPM       | 0.2              | 0.2               | 70%                    |
| **Relay Feedback**  | Water Level Control          | A0        | 6          | 50.0 cm        | 0.15             | 0.15              | 90%                    |
| **IMC**             | Pressure Control             | A0        | 9          | 100.0 kPa      | 0.1              | 0.1               | 80%                    |
| **Tyreus-Luyben**   | Chemical Reactor Temperature | A0        | 10         | 80.0°C         | 0.1              | 0.1               | 80%                    |
| **Manual Tuning**   | Generic Control System       | A0        | 11         | 50.0 (Arbitrary)| 0.1              | 0.1               | 80%                    |

---

This updated `usage.md` provides **individual examples** for each tuning algorithm, including proper pinouts, setpoints, **filtering**, and **anti-windup**. Let me know if you need further adjustments!