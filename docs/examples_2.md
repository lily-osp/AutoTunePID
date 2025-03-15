# V1.2.0 examples

---

## 1. Basic Example: Temperature Control

This example demonstrates a **basic temperature control system** using the `AutoTunePID` library. It uses the **Ziegler-Nichols** tuning method and includes **input filtering** and **anti-windup**.

### Code

```cpp
#include <AutoTunePID.h>

// Initialize PID controller with output range and tuning method
AutoTunePID pid(0, 255, TuningMethod::ZieglerNichols);

// Pin definitions
const int TEMP_SENSOR_PIN = A0; // Temperature sensor input
const int HEATER_PIN = 9;       // Heater output (PWM)

void setup() {
  // Set the target temperature (setpoint)
  pid.setSetpoint(75.0);

  // Enable input filtering with alpha = 0.1
  pid.enableInputFilter(0.1);

  // Enable anti-windup with 80% threshold
  pid.enableAntiWindup(true, 0.8);

  // Set oscillation mode to Normal
  pid.setOscillationMode(OscillationMode::Normal);

  // Start in Tune mode to auto-tune the PID parameters
  pid.setOperationalMode(OperationalMode::Tune);
}

void loop() {
  // Read the temperature sensor
  float temperature = analogRead(TEMP_SENSOR_PIN) * 0.48828125; // Convert ADC to °C

  // Update the PID controller
  pid.update(temperature);

  // Write the output to the heater (PWM)
  analogWrite(HEATER_PIN, pid.getOutput());

  // Small delay for stability
  delay(100);
}
```

### Explanation

- **Setpoint**: The target temperature is set to 75°C.
- **Input Filtering**: The input signal from the temperature sensor is filtered to reduce noise.
- **Anti-Windup**: Prevents integral windup when the heater output is saturated.
- **Auto-Tuning**: The system starts in **Tune mode** to automatically determine the PID gains using the Ziegler-Nichols method.

---

## 2. Intermediate Example: Motor Speed Control

This example demonstrates a **motor speed control system** using the `AutoTunePID` library. It uses the **Cohen-Coon** tuning method and includes **output filtering**, **system corrector**, and **natural oscillation detection**.

### Code

```cpp
#include <AutoTunePID.h>

// Initialize PID controller with output range and tuning method
AutoTunePID pid(0, 255, TuningMethod::CohenCoon);

// Pin definitions
const int ENCODER_PIN = A0; // Encoder input for motor speed
const int MOTOR_PIN = 9;    // Motor output (PWM)

void setup() {
  // Set the target motor speed (setpoint)
  pid.setSetpoint(1500); // Target RPM

  // Enable output filtering with alpha = 0.2
  pid.enableOutputFilter(0.2);

  // Enable system corrector with data window size = 10 and stability threshold = 5.0
  pid.enableCorrector(true, 10, 5.0);

  // Set oscillation mode to Half
  pid.setOscillationMode(OscillationMode::Half);

  // Start in Tune mode to auto-tune the PID parameters
  pid.setOperationalMode(OperationalMode::Tune);
}

void loop() {
  // Read the motor speed from the encoder
  float motorSpeed = readEncoderSpeed(); // Function to read encoder and calculate RPM

  // Update the PID controller
  pid.update(motorSpeed);

  // Write the output to the motor (PWM)
  analogWrite(MOTOR_PIN, pid.getOutput());

  // Small delay for stability
  delay(100);
}

// Function to read encoder and calculate RPM
float readEncoderSpeed() {
  // Placeholder for encoder reading logic
  return analogRead(ENCODER_PIN) * 2.0; // Example conversion to RPM
}
```

### Explanation

- **Setpoint**: The target motor speed is set to 1500 RPM.
- **Output Filtering**: The output signal to the motor is filtered to smooth the control action.
- **System Corrector**: Monitors the system for instability and applies corrective actions if needed.
- **Natural Oscillation Detection**: The system uses natural oscillations to auto-tune the PID gains.
- **Auto-Tuning**: The system starts in **Tune mode** to automatically determine the PID gains using the Cohen-Coon method.

---

## 3. Advanced Example: Chemical Reactor Temperature Control

This example demonstrates an **advanced chemical reactor temperature control system** using the `AutoTunePID` library. It uses the **Lambda Tuning (CLD)** method and includes **input/output filtering**, **anti-windup**, **system corrector**, and **multiple operational modes**.

### Code

```cpp
#include <AutoTunePID.h>

// Initialize PID controller with output range and tuning method
AutoTunePID pid(0, 255, TuningMethod::LambdaTuning);

// Pin definitions
const int TEMP_SENSOR_PIN = A0; // Temperature sensor input
const int HEATER_PIN = 9;       // Heater output (PWM)
const int COOLER_PIN = 10;      // Cooler output (PWM)

void setup() {
  // Set the target temperature (setpoint)
  pid.setSetpoint(80.0); // Target reactor temperature

  // Enable input and output filtering with alpha = 0.1
  pid.enableInputFilter(0.1);
  pid.enableOutputFilter(0.1);

  // Enable anti-windup with 90% threshold
  pid.enableAntiWindup(true, 0.9);

  // Enable system corrector with data window size = 15 and stability threshold = 3.0
  pid.enableCorrector(true, 15, 3.0);

  // Set oscillation mode to Mild
  pid.setOscillationMode(OscillationMode::Mild);

  // Set lambda parameter for Lambda Tuning
  pid.setLambda(0.5);

  // Start in Tune mode to auto-tune the PID parameters
  pid.setOperationalMode(OperationalMode::Tune);
}

void loop() {
  // Read the reactor temperature
  float temperature = analogRead(TEMP_SENSOR_PIN) * 0.48828125; // Convert ADC to °C

  // Update the PID controller
  pid.update(temperature);

  // Write the output to the heater or cooler (PWM)
  float output = pid.getOutput();
  if (output >= 0) {
    analogWrite(HEATER_PIN, output);
    analogWrite(COOLER_PIN, 0); // Turn off cooler
  } else {
    analogWrite(COOLER_PIN, -output);
    analogWrite(HEATER_PIN, 0); // Turn off heater
  }

  // Small delay for stability
  delay(100);
}
```

### Explanation

- **Setpoint**: The target reactor temperature is set to 80°C.
- **Input/Output Filtering**: Both the input and output signals are filtered to reduce noise and smooth control actions.
- **Anti-Windup**: Prevents integral windup when the heater or cooler output is saturated.
- **System Corrector**: Monitors the system for instability and applies corrective actions if needed.
- **Lambda Tuning**: Uses the **Lambda Tuning (CLD)** method to calculate PID gains, which is ideal for systems with significant dead time.
- **Multiple Operational Modes**: The system starts in **Tune mode** to auto-tune the PID gains and can switch to other modes (e.g., Normal, Hold) as needed.

---

## Summary of Examples

1. **Basic Example**: Temperature control using Ziegler-Nichols tuning, input filtering, and anti-windup.
2. **Intermediate Example**: Motor speed control using Cohen-Coon tuning, output filtering, system corrector, and natural oscillation detection.
3. **Advanced Example**: Chemical reactor temperature control using Lambda Tuning, input/output filtering, anti-windup, system corrector, and multiple operational modes.

These examples demonstrate how to use all the new features of the `AutoTunePID` library in different scenarios.

---

## Special case

> For systems that require **near oscillation-less control**, such as **autothrottle systems**, **auto thrusters**, or **precision positioning systems**, the `AutoTunePID` library can be configured to minimize oscillations while maintaining stability and responsiveness. Below is an **advanced example** for a system that requires near oscillation-less control, using **natural oscillation detection**, **system corrector**, and **Lambda Tuning (CLD)**.

---

## Example: Precision Positioning System

This example demonstrates a **precision positioning system** (e.g., for a robotic arm or CNC machine) that requires near oscillation-less control. The system uses **Lambda Tuning (CLD)** for robust tuning, **natural oscillation detection**, and **system corrector** to ensure stability and precision.

### Code

```cpp
#include <AutoTunePID.h>

// Initialize PID controller with output range and tuning method
AutoTunePID pid(-255, 255, TuningMethod::LambdaTuning); // Output range for bidirectional control

// Pin definitions
const int POSITION_SENSOR_PIN = A0; // Position sensor input (e.g., potentiometer or encoder)
const int MOTOR_PIN1 = 9;          // Motor output 1 (PWM)
const int MOTOR_PIN2 = 10;         // Motor output 2 (PWM)

void setup() {
  // Set the target position (setpoint)
  pid.setSetpoint(512); // Target position (e.g., 512 for midpoint of a 10-bit ADC)

  // Enable input and output filtering with alpha = 0.1
  pid.enableInputFilter(0.1);
  pid.enableOutputFilter(0.1);

  // Enable anti-windup with 90% threshold
  pid.enableAntiWindup(true, 0.9);

  // Enable system corrector with data window size = 20 and stability threshold = 1.0
  pid.enableCorrector(true, 20, 1.0);

  // Set oscillation mode to Mild (minimal oscillation)
  pid.setOscillationMode(OscillationMode::Mild);

  // Set lambda parameter for Lambda Tuning (CLD)
  pid.setLambda(0.2); // Smaller lambda for smoother response

  // Start in Tune mode to auto-tune the PID parameters
  pid.setOperationalMode(OperationalMode::Tune);
}

void loop() {
  // Read the current position from the sensor
  float currentPosition = analogRead(POSITION_SENSOR_PIN); // 10-bit ADC value (0-1023)

  // Update the PID controller
  pid.update(currentPosition);

  // Write the output to the motor (bidirectional control)
  float output = pid.getOutput();
  if (output >= 0) {
    analogWrite(MOTOR_PIN1, output);
    analogWrite(MOTOR_PIN2, 0); // Turn off reverse direction
  } else {
    analogWrite(MOTOR_PIN2, -output);
    analogWrite(MOTOR_PIN1, 0); // Turn off forward direction
  }

  // Small delay for stability
  delay(10); // Faster update rate for precision systems
}
```

---

### Explanation

1. **Setpoint**:
   - The target position is set to `512`, which corresponds to the midpoint of a 10-bit ADC (e.g., a potentiometer or encoder).

2. **Input/Output Filtering**:
   - Both the input (position sensor) and output (motor control) signals are filtered using an exponential moving average with `alpha = 0.1` to reduce noise and smooth the control action.

3. **Anti-Windup**:
   - Anti-windup is enabled with a threshold of `90%` to prevent the integral term from accumulating excessively when the output is saturated.

4. **System Corrector**:
   - The system corrector is enabled with a **data window size of 20** and a **stability threshold of 1.0**. This monitors the system's response for instability and applies corrective actions (e.g., reducing output or resetting the integral term) if needed.

5. **Oscillation Mode**:
   - The oscillation mode is set to **Mild**, which minimizes oscillations during auto-tuning and normal operation.

6. **Lambda Tuning (CLD)**:
   - The **Lambda Tuning (CLD)** method is used with a small `lambda` value (`0.2`) to ensure a smooth and stable response with minimal overshoot.

7. **Bidirectional Control**:
   - The motor is controlled bidirectionally using two PWM outputs (`MOTOR_PIN1` and `MOTOR_PIN2`). The PID output is split into forward and reverse directions to achieve precise positioning.

8. **Fast Update Rate**:
   - The system updates every `10 ms` to ensure rapid response and high precision.

---

### Key Features for Near Oscillation-Less Control

1. **Natural Oscillation Detection**:
   - The system uses **natural oscillation detection** during auto-tuning to minimize forced oscillations, making it suitable for systems that cannot tolerate aggressive tuning.

2. **System Corrector**:
   - The corrector ensures that the system remains stable even in the presence of disturbances or changes in dynamics.

3. **Lambda Tuning (CLD)**:
   - The **Lambda Tuning (CLD)** method is ideal for systems requiring smooth and stable control, as it balances response speed and robustness.

4. **Mild Oscillation Mode**:
   - The **Mild** oscillation mode ensures that the system operates with minimal oscillations during both auto-tuning and normal operation.

---

### Example Use Case

Imagine a **robotic arm** that needs to position itself precisely to pick and place objects. The system must avoid oscillations to ensure accurate positioning and prevent damage to the objects or the arm itself. This example demonstrates how the `AutoTunePID` library can be used to achieve near oscillation-less control in such a system.

---

### Performance Considerations

- **Update Rate**:
  - A fast update rate (`10 ms`) is used to ensure rapid response and high precision. Adjust this based on your system's requirements.

- **Lambda Value**:
  - The `lambda` value controls the trade-off between response speed and stability. A smaller `lambda` (e.g., `0.2`) results in a smoother response but may slow down the system's reaction time.

- **Stability Threshold**:
  - The stability threshold for the system corrector should be set based on the system's dynamics. A lower threshold (e.g., `1.0`) ensures that even small instabilities are detected and corrected.

---

This example demonstrates how to use the `AutoTunePID` library for systems requiring near oscillation-less control.

---
