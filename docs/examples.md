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
9. [Operational Modes Demonstration](#operational-modes-demonstration)
10. [Delta Robot Joint Control (High-Precision Position)](#delta-robot-joint-control-high-precision-position)
11. [Cascade Pressure/Flow Control (Industrial Master/Slave)](#cascade-pressureflow-control-industrial-masterslave)
12. [Smart CC/CV Battery Charger (State-Machine Integration)](#smart-cccv-battery-charger-state-machine-integration)
13. [Deterministic RTOS / Timer Interrupt Control](#deterministic-rtos--timer-interrupt-control)

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

## Operational Modes Demonstration

### System Flexibility and Control

This example shows how to use all available operational modes to achieve system flexibility, including **Normal**, **Reverse**, **Manual**, **Override**, **Track**, **Hold**, and **Preserve** modes.

#### Pin Configuration

- **Input Pin**: A0 (Sensor Input)
- **Output Pin**: 9 (PWM Control)

#### Code

```cpp
#include <AutoTunePID.h>

// Use the atp namespace
using namespace atp;

// Pin definitions
const int sensorPin = A0;    // Analog input for process variable
const int outputPin = 9;     // PWM output for control signal

// PID controller instance
AutoTunePID pid(0.0f, 255.0f, TuningMethod::ZieglerNichols);

void setup() {
    Serial.begin(9600);
    pinMode(sensorPin, INPUT);
    pinMode(outputPin, OUTPUT);

    // Configure PID for normal operation
    pid.setSetpoint(50.0f); // Target value of 50 (analog units)
    pid.setManualGains(2.0f, 0.5f, 0.1f); // Sample PID gains
}

void loop() {
    // Read process variable (0-1023 range)
    float processVariable = static_cast<float>(analogRead(sensorPin)) * (100.0f / 1023.0f);

    // Update PID controller
    pid.update(processVariable);

    // Get output and apply to control pin
    float output = pid.getOutput();
    analogWrite(outputPin, static_cast<int>(output));

    // Handle serial commands to change modes (Simulated)
    if (Serial.available()) {
        char command = Serial.read();
        switch(command) {
            case 'n': pid.setOperationalMode(OperationalMode::Normal); break;
            case 'r': pid.setOperationalMode(OperationalMode::Reverse); break;
            case 'h': pid.setOperationalMode(OperationalMode::Hold); break;
            // Additional modes can be set similarly
        }
    }
    delay(100);
}
```

---

## Delta Robot Joint Control (High-Precision Position)

### Handling Gearbox Friction and Inertia

Controlling a Delta Robot joint presents a significant challenge due to **gearbox friction** and the varying **inertia** of the robot arm. This example utilizes the **Cohen-Coon** tuning method, which is particularly effective for systems with physical lag and friction. By using high-precision position control (in degrees) and specialized tuning, the system achieves smooth and accurate movements.

#### Key Features
- **Cohen-Coon Tuning**: Optimized for systems with significant dead time and friction.
- **Position Control**: Direct mapping of encoder ticks to degrees.
- **Mild Oscillation**: Safe auto-tuning within a restricted range (+/- 15 degrees).
- **Bidirectional Output**: Handles motor direction and PWM for full range movement (-255 to 255).

#### Pin Configuration

- **Encoder Pin A**: 2 (Interrupt-capable)
- **Encoder Pin B**: 3
- **Motor PWM Pin**: 10
- **Motor Dir Pin**: 7
- **Setpoint**: 45.0°

#### Code

```cpp
#include <AutoTunePID.h>

// Use the atp namespace for AUTOSAR compliance
using namespace atp;

// Hardware Configuration
const int ENCODER_PIN_A = 2;
const int ENCODER_PIN_B = 3;
const int MOTOR_PWM_PIN = 10;
const int MOTOR_DIR_PIN = 7;

// System Parameters
const float PULSES_PER_DEGREE = 14.22f; 
volatile int32_t encoderTicks = 0;
float targetAngle = 45.0f;

// PID Controller Instance
// Range: -255.0 to 255.0 (Bidirectional PWM)
// Method: CohenCoon (Excellent for high-inertia/friction robotic joints)
AutoTunePID jointPID(-255.0f, 255.0f, TuningMethod::CohenCoon);

void handleEncoder() {
    if (digitalRead(ENCODER_PIN_B) == HIGH) {
        encoderTicks++;
    } else {
        encoderTicks--;
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), handleEncoder, RISING);

    jointPID.setSetpoint(targetAngle);
    jointPID.enableInputFilter(0.15f);      // Smooth out noisy encoder pulses
    jointPID.enableAntiWindup(true, 0.85f); // Prevent integral runaway
    jointPID.setOscillationMode(OscillationMode::Mild); // Safety: restricted range
    jointPID.setOperationalMode(OperationalMode::Tune); // Find best gains for friction
}

void loop() {
    static uint32_t lastLoop = 0U;
    uint32_t now = millis();

    if ((now - lastLoop) >= 100U) {
        lastLoop = now;
        float currentAngle = static_cast<float>(encoderTicks) / PULSES_PER_DEGREE;

        jointPID.update(currentAngle);
        float pidOutput = jointPID.getOutput();
        
        // Handle bidirectional motor driver
        if (pidOutput >= 0.0f) {
            digitalWrite(MOTOR_DIR_PIN, HIGH);
            analogWrite(MOTOR_PWM_PIN, static_cast<int>(pidOutput));
        } else {
            digitalWrite(MOTOR_DIR_PIN, LOW);
            analogWrite(MOTOR_PWM_PIN, static_cast<int>(fabsf(pidOutput)));
        }
    }
}
```

---

## Cascade Pressure/Flow Control (Industrial Master/Slave)

### Two-Stage Coordinated Control

In complex industrial systems, a single PID loop may respond too slowly to supply-side disturbances. This example demonstrates **Cascade Control**, where two PID instances work in a master/slave relationship:
1. **Master PID (Pressure)**: Monitors pipe pressure and calculates the required Flow Rate.
2. **Slave PID (Flow)**: Monitors actual flow and calculates the Motor PWM to achieve the master's target flow rate.

This pattern provides much faster response to pressure drops or pump fluctuations.

#### Pin Configuration

- **Pressure Sensor**: A0
- **Flow Sensor**: A1
- **Pump PWM**: 9
- **Setpoint**: 50.0 PSI

#### Code

```cpp
#include <AutoTunePID.h>

using namespace atp;

// Master PID: Pressure Control (Input: PSI, Output: Target Flow LPM)
AutoTunePID pressureMaster(0.0f, 100.0f, TuningMethod::IMC);

// Slave PID: Flow Control (Input: LPM, Output: Pump PWM 0-255)
AutoTunePID flowSlave(0.0f, 255.0f, TuningMethod::CohenCoon);

void setup() {
    pressureMaster.setSetpoint(50.0f);
    pressureMaster.enableInputFilter(0.1f);
    pressureMaster.enableAntiWindup(true, 0.8f);

    flowSlave.enableInputFilter(0.2f);
    flowSlave.enableAntiWindup(true, 0.9f);

    flowSlave.setOperationalMode(OperationalMode::Normal);
    pressureMaster.setOperationalMode(OperationalMode::Normal);
}

void loop() {
    static uint32_t lastUpdate = 0U;
    if ((millis() - lastUpdate) >= 100U) {
        lastUpdate = millis();

        float currentPressure = static_cast<float>(analogRead(A0)) * (100.0f / 1023.0f);
        float currentFlow = static_cast<float>(analogRead(A1)) * (150.0f / 1023.0f);

        // Master Loop
        pressureMaster.update(currentPressure);
        float targetFlow = pressureMaster.getOutput();

        // Update Slave Setpoint and Loop
        flowSlave.setSetpoint(targetFlow);
        flowSlave.update(currentFlow);
        
        analogWrite(9, static_cast<int>(flowSlave.getOutput()));
    }
}
```

---

## Smart CC/CV Battery Charger (State-Machine Integration)

### High-Efficiency Power Management

This example demonstrates a professional **Constant Current / Constant Voltage (CC/CV)** charger. It showcases how to transition between different PID controllers and operational modes:
1. **CC Phase**: Uses a PID to maintain a constant 5A charging current.
2. **CV Phase**: Once the battery reaches 14.4V, it switches to a Voltage PID.
3. **Safety**: Uses `Override` mode for emergency shutoff and handles smooth handovers.

#### Pin Configuration

- **Voltage Sense**: A0 (0-20V range)
- **Current Sense**: A1 (0-10A range)
- **Charger PWM**: 9
- **Target Voltage**: 14.4V

#### Code

```cpp
#include <AutoTunePID.h>

using namespace atp;

AutoTunePID ccPID(0.0f, 255.0f, TuningMethod::TyreusLuyben); // Current Limit
AutoTunePID cvPID(0.0f, 255.0f, TuningMethod::IMC);          // Voltage Target

enum class State { CC, CV, FULL };
State chargerState = State::CC;

void setup() {
    ccPID.setSetpoint(5.0f); // 5 Amps
    cvPID.setSetpoint(14.4f); // 14.4 Volts
    ccPID.setOperationalMode(OperationalMode::Normal);
}

void loop() {
    static uint32_t lastUpdate = 0U;
    if ((millis() - lastUpdate) >= 100U) {
        lastUpdate = millis();

        float battV = static_cast<float>(analogRead(A0)) * (20.0f / 1023.0f);
        float chargeI = static_cast<float>(analogRead(A1)) * (10.0f / 1023.0f);

        if (chargerState == State::CC) {
            ccPID.update(chargeI);
            analogWrite(9, static_cast<int>(ccPID.getOutput()));
            if (battV >= 14.4f) chargerState = State::CV;
        } 
        else if (chargerState == State::CV) {
            cvPID.update(battV);
            analogWrite(9, static_cast<int>(cvPID.getOutput()));
            if (chargeI < 0.5f) {
                chargerState = State::FULL;
                analogWrite(9, 0);
            }
        }
    }
}
```

---

## Deterministic RTOS / Timer Interrupt Control

### Explicit Delta-Time API for Jitter-Free Execution

Standard Arduino control loops using `millis()` can suffer from jitter due to other blocking tasks (e.g., Serial communication, displays). This example shows how to use the overloaded `update(input, dt)` method inside a hardware timer interrupt (or an RTOS task) to guarantee execution at a fixed interval, bypassing `millis()` entirely for true deterministic timing.

#### Pin Configuration

- **Input Pin**: A0 (Sensor)
- **Output Pin**: 9 (Actuator)
- **Setpoint**: 100.0

#### Code

```cpp
#include <AutoTunePID.h>

using namespace atp;

// --- Hardware Configuration ---
const int SENSOR_PIN = A0;
const int ACTUATOR_PIN = 9;

// --- PID Controller Instance ---
AutoTunePID precisionPID(0.0f, 255.0f, TuningMethod::ZieglerNichols);

// --- State Variables ---
volatile float currentSensorValue = 0.0f;
volatile float currentOutput = 0.0f;
volatile bool newOutputAvailable = false;

// 50ms interval (0.05 seconds)
const float DT_SECONDS = 0.05f; 

void setup() {
    Serial.begin(115200);
    pinMode(ACTUATOR_PIN, OUTPUT);

    precisionPID.setSetpoint(100.0f);
    
    // Setup Timer1 to fire an interrupt every 50ms (AVR specific example)
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A = 3125; // Compare match register for 50ms (20Hz) at 16MHz with 256 prescaler
    TCCR1B |= (1 << WGM12); // CTC mode
    TCCR1B |= (1 << CS12);  // 256 prescaler
    TIMSK1 |= (1 << OCIE1A); // Enable timer compare interrupt
    interrupts();

    Serial.println("--- Deterministic RTOS/ISR Controller Started ---");
}

// Timer1 Interrupt Service Routine - Fires exactly every 50ms
ISR(TIMER1_COMPA_vect) {
    int rawVal = analogRead(SENSOR_PIN);
    currentSensorValue = static_cast<float>(rawVal) * (200.0f / 1023.0f);

    // Deterministic PID Update using explicit dt
    precisionPID.update(currentSensorValue, DT_SECONDS);
    
    currentOutput = precisionPID.getOutput();
    newOutputAvailable = true;
}

void loop() {
    if (newOutputAvailable) {
        noInterrupts();
        float outputToApply = currentOutput;
        float sensorToPrint = currentSensorValue;
        newOutputAvailable = false;
        interrupts();

        analogWrite(ACTUATOR_PIN, static_cast<int>(outputToApply));

        Serial.print("Sensor: "); Serial.print(sensorToPrint);
        Serial.print(" | PWM: "); Serial.println(outputToApply);
    }
}
```

---

## Summary of Examples with Filtering, Anti-Windup, and Oscillation Modes

| Tuning Method | Example Application | Input Pin | Output Pin | Setpoint | Filter (α) | Anti-Windup | Mode |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Ziegler-Nichols** | Temperature Control | A0 | 3 | 75.0°C | 0.1 | 80% | Tune |
| **Cohen-Coon** | Motor Speed Control | A0 | 5 | 1500 RPM | 0.2 | 70% | Tune |
| **IMC** | Pressure Control | A0 | 9 | 100.0 kPa | 0.1 | 80% | Tune |
| **Tyreus-Luyben** | Reactor Temperature | A0 | 10 | 80.0°C | 0.1 | 80% | Tune |
| **Lambda Tuning** | Flow Control | A0 | 11 | 50.0 L/min | 0.15 | 90% | Tune |
| **Manual Tuning** | Generic System | A0 | 12 | 50.0 Unit | 0.1 | 80% | Normal |
| **Booster Motor** | Motor Speed Booster | 2 (Enc) | 9 | 1500.0 RPM | 0.2 | 90% | Tune |
| **Peltier Cooling**| Cooling (Reverse) | A0 | 9 | 12.0°C | 0.1 | 80% | Reverse |
| **Ziegler-Nichols** | Operational Modes | A0 | 9 | 50.0 Unit | N/A | N/A | Multiple |
| **Cohen-Coon** | Delta Robot Joint | 2 (Enc) | 10 | 45.0 Deg | 0.15 | 85% | Tune |
| **Cascade Control**| Pressure/Flow | A0, A1 | 9 | 50.0 PSI | 0.1, 0.2 | 80%, 90% | Normal |
| **CC/CV Charger** | Battery Charger | A0, A1 | 9 | 14.4V, 5A | N/A | 85%, 90% | CC/CV |
| **Explicit dt API** | Deterministic RTOS/ISR | A0 | 9 | 100.0 | N/A | N/A | Normal |

---
