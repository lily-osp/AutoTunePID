# Using the AutoTunePID Library

The `AutoTunePID` library is a powerful tool for adaptive PID control in Arduino projects. It features automatic tuning based on methods like **Ziegler-Nichols**, **Cohen-Coon**, **IMC**, **Tyreus-Luyben**, and **Lambda Tuning (CLD)**, as well as manual tuning options. This guide provides detailed examples for each tuning method, including proper pinouts, setpoints, **filtering**, **anti-windup**, **operational modes**, and **oscillation modes**.

---

## Table of Contents

1. [Ziegler-Nichols Example with Filtering, Anti-Windup, and Oscillation Modes](#ziegler-nichols-example-with-filtering-anti-windup-and-oscillation-modes)
2. [Cohen-Coon Example with Filtering, Anti-Windup, and Oscillation Modes](#cohen-coon-example-with-filtering-anti-windup-and-oscillation-modes)
3. [IMC Example with Filtering, Anti-Windup, and Oscillation Modes](#imc-example-with-filtering-anti-windup-and-oscillation-modes)
4. [Tyreus-Luyben Example with Filtering, Anti-Windup, and Oscillation Modes](#tyreus-luyben-example-with-filtering-anti-windup-and-oscillation-modes)
5. [Lambda Tuning Example with Filtering, Anti-Windup, and Oscillation Modes](#lambda-tuning-example-with-filtering-anti-windup-and-oscillation-modes)
6. [Manual Tuning Example with Filtering, Anti-Windup, and Oscillation Modes](#manual-tuning-example-with-filtering-anti-windup-and-oscillation-modes)

---

## Ziegler-Nichols Example with Filtering, Anti-Windup, and Oscillation Modes

### Temperature Control System

This example demonstrates how to use the **Ziegler-Nichols** tuning method for a temperature control system with **input and output filtering**, **anti-windup**, and **oscillation modes**.

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
    tempController.setOscillationMode(OscillationMode::Normal); // Set oscillation mode to Normal
    tempController.setOperationalMode(OperationalMode::Tune); // Set operational mode to Tune
}

void loop() {
    float temp = analogRead(A0) * (100.0 / 1023.0); // Read temperature sensor (0-100°C range)
    tempController.update(temp); // Update PID controller
    analogWrite(3, tempController.getOutput()); // Control heater
    delay(100); // Update every 100ms
}
```

---

## Cohen-Coon Example with Filtering, Anti-Windup, and Oscillation Modes

### Motor Speed Control System

This example demonstrates how to use the **Cohen-Coon** tuning method for a motor speed control system with **input and output filtering**, **anti-windup**, and **oscillation modes**.

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
    motorController.setOscillationMode(OscillationMode::Half); // Set oscillation mode to Half
    motorController.setOperationalMode(OperationalMode::Tune); // Set operational mode to Tune
}

void loop() {
    float rpm = analogRead(A0) * (3000.0 / 1023.0); // Read RPM (0-3000 RPM range)
    motorController.update(rpm); // Update PID controller
    analogWrite(5, motorController.getOutput()); // Control motor speed
    delay(100); // Update every 100ms
}
```

---

## IMC Example with Filtering, Anti-Windup, and Oscillation Modes

### Pressure Control System

This example demonstrates how to use the **IMC** tuning method for a pressure control system with **input and output filtering**, **anti-windup**, and **oscillation modes**.

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
    pressureController.setOscillationMode(OscillationMode::Normal); // Set oscillation mode to Normal
    pressureController.setOperationalMode(OperationalMode::Tune); // Set operational mode to Tune
}

void loop() {
    float pressure = analogRead(A0) * (200.0 / 1023.0); // Read pressure (0-200 kPa range)
    pressureController.update(pressure); // Update PID controller
    analogWrite(9, pressureController.getOutput()); // Control pump
    delay(100); // Update every 100ms
}
```

---

## Tyreus-Luyben Example with Filtering, Anti-Windup, and Oscillation Modes

### Chemical Reactor Temperature Control

This example demonstrates how to use the **Tyreus-Luyben** tuning method for a chemical reactor temperature control system with **input and output filtering**, **anti-windup**, and **oscillation modes**.

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
    reactorController.setOscillationMode(OscillationMode::Half); // Set oscillation mode to Half
    reactorController.setOperationalMode(OperationalMode::Tune); // Set operational mode to Tune
}

void loop() {
    float temp = analogRead(A0) * (100.0 / 1023.0); // Read temperature (0-100°C range)
    reactorController.update(temp); // Update PID controller
    analogWrite(10, reactorController.getOutput()); // Control heater
    delay(100); // Update every 100ms
}
```

---

## Lambda Tuning Example with Filtering, Anti-Windup, and Oscillation Modes

### Flow Control System

This example demonstrates how to use the **Lambda Tuning (CLD)** method for a flow control system with **input and output filtering**, **anti-windup**, and **oscillation modes**.

#### Pin Configuration

- **Input Pin**: A0 (Flow Sensor)
- **Output Pin**: 11 (Valve Control)
- **Setpoint**: 50.0 L/min

#### Code

```cpp
#include "AutoTunePID.h"

// Initialize PID controller with output range and Lambda Tuning method
AutoTunePID flowController(0, 255, TuningMethod::LambdaTuning);

void setup() {
    flowController.setSetpoint(50.0); // Set target flow rate to 50 L/min
    flowController.enableInputFilter(0.15);  // Enable input filtering with alpha = 0.15
    flowController.enableOutputFilter(0.15); // Enable output filtering with alpha = 0.15
    flowController.enableAntiWindup(true, 0.9); // Enable anti-windup with 90% threshold
    flowController.setOscillationMode(OscillationMode::Mild); // Set oscillation mode to Mild
    flowController.setOperationalMode(OperationalMode::Tune); // Set operational mode to Tune
}

void loop() {
    float flowRate = analogRead(A0) * (100.0 / 1023.0); // Read flow rate (0-100 L/min range)
    flowController.update(flowRate); // Update PID controller
    analogWrite(11, flowController.getOutput()); // Control valve
    delay(100); // Update every 100ms
}
```

---

## Manual Tuning Example with Filtering, Anti-Windup, and Oscillation Modes

### Generic Control System

This example demonstrates how to use **manual tuning** for a generic control system with **input and output filtering**, **anti-windup**, and **oscillation modes**.

#### Pin Configuration

- **Input Pin**: A0 (Sensor Input)
- **Output Pin**: 12 (Actuator Control)
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
    manualController.setOscillationMode(OscillationMode::Normal); // Set oscillation mode to Normal
    manualController.setOperationalMode(OperationalMode::Normal); // Set operational mode to Normal
}

void loop() {
    float input = analogRead(A0) * (100.0 / 1023.0); // Read sensor input (0-100 range)
    manualController.update(input); // Update PID controller
    analogWrite(12, manualController.getOutput()); // Control actuator
    delay(100); // Update every 100ms
}
```

---

## Summary of Examples with Filtering, Anti-Windup, and Oscillation Modes

| Tuning Method       | Example Application          | Input Pin | Output Pin | Setpoint         | Input Filter (α) | Output Filter (α) | Anti-Windup Threshold | Oscillation Mode | Operational Mode |
| ------------------- | ---------------------------- | --------- | ---------- | ---------------- | ---------------- | ----------------- | --------------------- | ---------------- | ---------------- |
| **Ziegler-Nichols** | Temperature Control          | A0        | 3          | 75.0°C           | 0.1              | 0.1               | 80%                   | Normal           | Tune             |
| **Cohen-Coon**      | Motor Speed Control          | A0        | 5          | 1500 RPM         | 0.2              | 0.2               | 70%                   | Half             | Tune             |
| **IMC**             | Pressure Control             | A0        | 9          | 100.0 kPa        | 0.1              | 0.1               | 80%                   | Normal           | Tune             |
| **Tyreus-Luyben**   | Chemical Reactor Temperature | A0        | 10         | 80.0°C           | 0.1              | 0.1               | 80%                   | Half             | Tune             |
| **Lambda Tuning**   | Flow Control                 | A0        | 11         | 50.0 L/min       | 0.15             | 0.15              | 90%                   | Mild             | Tune             |
| **Manual Tuning**   | Generic Control System       | A0        | 12         | 50.0 (Arbitrary) | 0.1              | 0.1               | 80%                   | Normal           | Normal           |

---
