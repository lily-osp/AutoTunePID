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
7. [Booster Motor Speed Controller (High-Performance Example)](#booster-motor-speed-controller-high-performance-example)
8. [Peltier Cooling Example with Reverse Mode](#peltier-cooling-example-with-reverse-mode)

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
#include <AutoTunePID.h>

using namespace atp;

// Initialize PID controller with output range and Ziegler-Nichols method
AutoTunePID tempController(0.0f, 255.0f, TuningMethod::ZieglerNichols);

void setup() {
    tempController.setSetpoint(75.0f); // Set target temperature to 75°C
    tempController.enableInputFilter(0.1f);  // Enable input filtering with alpha = 0.1
    tempController.enableOutputFilter(0.1f); // Enable output filtering with alpha = 0.1
    tempController.enableAntiWindup(true, 0.8f); // Enable anti-windup with 80% threshold
    tempController.setOscillationMode(OscillationMode::Normal); // Set oscillation mode to Normal
    tempController.setOperationalMode(OperationalMode::Tune); // Set operational mode to Tune
}

void loop() {
    // Read temperature sensor (0-100°C range)
    float temp = static_cast<float>(analogRead(A0)) * (100.0f / 1023.0f); 
    tempController.update(temp); // Update PID controller
    analogWrite(3, static_cast<int>(tempController.getOutput())); // Control heater
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
#include <AutoTunePID.h>

using namespace atp;

// Initialize PID controller with output range and Cohen-Coon method
AutoTunePID motorController(0.0f, 255.0f, TuningMethod::CohenCoon);

void setup() {
    motorController.setSetpoint(1500.0f); // Set target speed to 1500 RPM
    motorController.enableInputFilter(0.2f);  // Enable input filtering with alpha = 0.2
    motorController.enableOutputFilter(0.2f); // Enable output filtering with alpha = 0.2
    motorController.enableAntiWindup(true, 0.7f); // Enable anti-windup with 70% threshold
    motorController.setOscillationMode(OscillationMode::Half); // Set oscillation mode to Half
    motorController.setOperationalMode(OperationalMode::Tune); // Set operational mode to Tune
}

void loop() {
    // Read RPM (0-3000 RPM range)
    float rpm = static_cast<float>(analogRead(A0)) * (3000.0f / 1023.0f); 
    motorController.update(rpm); // Update PID controller
    analogWrite(5, static_cast<int>(motorController.getOutput())); // Control motor speed
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
#include <AutoTunePID.h>

using namespace atp;

// Initialize PID controller with output range and IMC method
AutoTunePID pressureController(0.0f, 255.0f, TuningMethod::IMC);

void setup() {
    pressureController.setSetpoint(100.0f); // Set target pressure to 100 kPa
    pressureController.enableInputFilter(0.1f);  // Enable input filtering with alpha = 0.1
    pressureController.enableOutputFilter(0.1f); // Enable output filtering with alpha = 0.1
    pressureController.enableAntiWindup(true, 0.8f); // Enable anti-windup with 80% threshold
    pressureController.setLambda(0.5f); // Set lambda parameter for IMC tuning
    pressureController.setOscillationMode(OscillationMode::Normal); // Set oscillation mode to Normal
    pressureController.setOperationalMode(OperationalMode::Tune); // Set operational mode to Tune
}

void loop() {
    // Read pressure (0-200 kPa range)
    float pressure = static_cast<float>(analogRead(A0)) * (200.0f / 1023.0f); 
    pressureController.update(pressure); // Update PID controller
    analogWrite(9, static_cast<int>(pressureController.getOutput())); // Control pump
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
#include <AutoTunePID.h>

using namespace atp;

// Initialize PID controller with output range and Tyreus-Luyben method
AutoTunePID reactorController(0.0f, 255.0f, TuningMethod::TyreusLuyben);

void setup() {
    reactorController.setSetpoint(80.0f); // Set target temperature to 80°C
    reactorController.enableInputFilter(0.1f);  // Enable input filtering with alpha = 0.1
    reactorController.enableOutputFilter(0.1f); // Enable output filtering with alpha = 0.1
    reactorController.enableAntiWindup(true, 0.8f); // Enable anti-windup with 80% threshold
    reactorController.setOscillationMode(OscillationMode::Half); // Set oscillation mode to Half
    reactorController.setOperationalMode(OperationalMode::Tune); // Set operational mode to Tune
}

void loop() {
    // Read temperature (0-100°C range)
    float temp = static_cast<float>(analogRead(A0)) * (100.0f / 1023.0f); 
    reactorController.update(temp); // Update PID controller
    analogWrite(10, static_cast<int>(reactorController.getOutput())); // Control heater
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
#include <AutoTunePID.h>

using namespace atp;

// Initialize PID controller with output range and Lambda Tuning method
AutoTunePID flowController(0.0f, 255.0f, TuningMethod::LambdaTuning);

void setup() {
    flowController.setSetpoint(50.0f); // Set target flow rate to 50 L/min
    flowController.enableInputFilter(0.15f);  // Enable input filtering with alpha = 0.15
    flowController.enableOutputFilter(0.15f); // Enable output filtering with alpha = 0.15
    flowController.enableAntiWindup(true, 0.9f); // Enable anti-windup with 90% threshold
    flowController.setLambda(0.7f); // Set lambda parameter for Lambda Tuning
    flowController.setOscillationMode(OscillationMode::Mild); // Set oscillation mode to Mild
    flowController.setOperationalMode(OperationalMode::Tune); // Set operational mode to Tune
}

void loop() {
    // Read flow rate (0-100 L/min range)
    float flowRate = static_cast<float>(analogRead(A0)) * (100.0f / 1023.0f); 
    flowController.update(flowRate); // Update PID controller
    analogWrite(11, static_cast<int>(flowController.getOutput())); // Control valve
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
#include <AutoTunePID.h>

using namespace atp;

// Initialize PID controller with output range and manual tuning
AutoTunePID manualController(0.0f, 255.0f, TuningMethod::Manual);

void setup() {
    manualController.setSetpoint(50.0f); // Set target value to 50
    manualController.setManualGains(1.0f, 0.5f, 0.1f); // Set Kp, Ki, Kd manually
    manualController.enableInputFilter(0.1f);  // Enable input filtering with alpha = 0.1
    manualController.enableOutputFilter(0.1f); // Enable output filtering with alpha = 0.1
    manualController.enableAntiWindup(true, 0.8f); // Enable anti-windup with 80% threshold
    manualController.setOscillationMode(OscillationMode::Normal); // Set oscillation mode to Normal
    manualController.setOperationalMode(OperationalMode::Normal); // Set operational mode to Normal
}

void loop() {
    // Read sensor input (0-100 range)
    float input = static_cast<float>(analogRead(A0)) * (100.0f / 1023.0f); 
    manualController.update(input); // Update PID controller
    analogWrite(12, static_cast<int>(manualController.getOutput())); // Control actuator
    delay(100); // Update every 100ms
}
```

---

## Booster Motor Speed Controller (High-Performance Example)

### Electronic Motor Booster System

This professional example demonstrates using the `AutoTunePID` library to control a high-speed motor. It utilizes **input filtering** for noisy encoder data and **anti-windup** for physical motor limits. The system starts in 'Tune' mode to identify optimal **Cohen-Coon** gains.

#### Pin Configuration

- **Encoder Pin**: 2 (Interrupt-capable)
- **Motor PWM Pin**: 9 (Speed Control)
- **Motor Dir Pin**: 8 (Direction)
- **Setpoint**: 1500.0 RPM

#### Code

```cpp
#include <AutoTunePID.h>

using namespace atp;

// Hardware Configuration
const int ENCODER_PIN = 2;
const int MOTOR_PWM_PIN = 9;
const int MOTOR_DIR_PIN = 8;

// System Parameters
float targetRPM = 1500.0f;
volatile uint32_t pulses = 0U;

// PID Controller Instance
// Range: 0.0 to 255.0 (Full Booster Power)
// Method: CohenCoon (Excellent for systems with physical lag/inertia)
AutoTunePID booster(0.0f, 255.0f, TuningMethod::CohenCoon);

void countPulse() { pulses++; }

void setup() {
    Serial.begin(115200);
    pinMode(ENCODER_PIN, INPUT_PULLUP);
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countPulse, RISING);

    digitalWrite(MOTOR_DIR_PIN, HIGH);

    booster.setSetpoint(targetRPM);
    booster.enableInputFilter(0.2f);     // Smooth high-frequency encoder noise
    booster.enableAntiWindup(true, 0.9f); // Prevent integral runaway
    booster.setOscillationMode(OscillationMode::Half);
    booster.setOperationalMode(OperationalMode::Tune);
}

void loop() {
    static uint32_t lastMillis = 0U;
    uint32_t now = millis();

    if ((now - lastMillis) >= 100U) {
        // Calculate RPM (Assuming 20 pulses per revolution)
        float currentRPM = static_cast<float>(pulses * 600) / 20.0f;
        pulses = 0U;
        lastMillis = now;

        booster.update(currentRPM);
        analogWrite(MOTOR_PWM_PIN, static_cast<int>(booster.getOutput()));

        if (booster.getOperationalMode() == OperationalMode::Normal) {
            // System Stable
        }
    }
}
```

---

## Peltier Cooling Example with Reverse Mode

### Peltier Cell Temperature Control

This example demonstrates how to use the **Reverse mode** for cooling systems, specifically for controlling a Peltier cell to maintain a target temperature. In cooling applications, you want to activate cooling when the temperature exceeds the setpoint.

#### Pin Configuration

- **Input Pin**: A0 (Temperature Sensor)
- **Output Pin**: 9 (Peltier Cell PWM Control)
- **Setpoint**: 12.0°C

#### Code

```cpp
#include <AutoTunePID.h>

using namespace atp;

// Pin definitions
const int tempSensorPin = A0; 
const int peltierPin = 9;     

// PID parameters
float setpoint = 12.0f; 
AutoTunePID pid(0.0f, 255.0f, TuningMethod::ZieglerNichols); 

void setup() {
    Serial.begin(9600);
    pinMode(tempSensorPin, INPUT);
    pinMode(peltierPin, OUTPUT);

    // Configure PID controller for cooling system
    pid.setSetpoint(setpoint); 
    pid.enableInputFilter(0.1f);  
    pid.enableAntiWindup(true, 0.8f); 
    pid.setOperationalMode(OperationalMode::Reverse); // Use Reverse mode for cooling
}

void loop() {
    // Read temperature sensor (0-100°C range)
    float currentTemp = static_cast<float>(analogRead(tempSensorPin)) * (100.0f / 1023.0f);

    pid.update(currentTemp);
    float output = pid.getOutput();
    analogWrite(peltierPin, static_cast<int>(output));

    Serial.print("Temperature: ");
    Serial.print(currentTemp);
    Serial.print("°C | Output: ");
    Serial.println(output);

    delay(100);
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
| **Booster Motor**   | High-Speed Speed Booster     | 2 (Enc)   | 9          | 1500.0 RPM       | 0.2              | N/A               | 90%                   | Half             | Tune             |
| **Reverse Mode**    | Peltier Cooling              | A0        | 9          | 12.0°C           | 0.1              | N/A               | 80%                   | N/A              | Reverse          |

---
