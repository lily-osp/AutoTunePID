# AutoTunePID Library

## Overview

The AutoTunePID library is an easy-to-use library for Arduino IDE that provides a powerful PID (Proportional, Integral, Derivative) controller with built-in auto-tuning capabilities. This library allows developers to implement PID control in their projects with minimal setup, ensuring efficient and accurate system control. It supports a wide range of applications, such as temperature control, motor speed regulation, distance maintenance, and light intensity control.

---

## Features

- **PID Control:** Automatically adjusts output based on proportional, integral, and derivative error terms.
- **Auto-Tuning:** Dynamically calculates optimal PID parameters (Kp, Ki, Kd) using the Ziegler-Nichols method.
- **Configurable Parameters:** Set target values (setpoints), tuning duration, and system-specific constraints.
- **Versatile Applications:** Works with any system requiring feedback-based control (temperature, distance, speed, light, etc.).
- **Easy Integration:** Simple and intuitive API for effortless integration into Arduino sketches.
- **Examples Included:** Three comprehensive examples for quick deployment: temperature control, distance control, and light intensity control.

---

## Installation

1. **Download the Library:** Download the AutoTunePID library as a `.zip` file from the [GitHub repository](#).
2. **Add to Arduino IDE:**
   - Open the Arduino IDE.
   - Go to **Sketch > Include Library > Add .ZIP Library...**.
   - Select the downloaded `.zip` file.
3. **Restart Arduino IDE:** Restart the IDE to ensure the library is loaded.

---

## Library Files

### 1. `AutoTunePID.h`

Contains the class definition and public methods for using the library.

### 2. `AutoTunePID.cpp`

Implements the PID control logic, auto-tuning algorithm, and utility functions.

### 3. `library.properties`

Provides metadata for the Arduino IDE.

### 4. `examples/`

Includes example sketches for common applications:

- **TemperatureControl:** PID-based temperature control system.
- **DistanceControl:** PID-based distance regulation system.
- **LightIntensityControl:** PID-based light intensity control system.

---

## API Reference

### **Class:** `AutoTunePID`

#### **Constructor**

```cpp
AutoTunePID(float maxOutput, float minOutput, unsigned long tuningDuration);
```

- **maxOutput:** Maximum allowable output value.
- **minOutput:** Minimum allowable output value.
- **tuningDuration:** Duration (in milliseconds) for the auto-tuning process.

#### **Methods**

1. **`void setSetpoint(float setpoint);`**

   - Sets the desired target value for the system.

2. **`float compute(float input);`**

   - Computes the PID output based on the current input.
   - **Returns:** The control output.

3. **`void startTuning();`**

   - Initiates the auto-tuning process.

4. **`bool isTuning();`**

   - Checks if the system is still in the tuning phase.
   - **Returns:** `true` if tuning is in progress, `false` otherwise.

5. **`void reset();`**

   - Resets the PID controller, clearing integral and error terms.

---

## Usage

### **Basic Workflow**

1. Include the library:
   ```cpp
   #include <AutoTunePID.h>
   ```
2. Create an instance of the `AutoTunePID` class:
   ```cpp
   AutoTunePID pid(maxOutput, minOutput, tuningDuration);
   ```
3. Set the desired target value (setpoint):
   ```cpp
   pid.setSetpoint(targetValue);
   ```
4. In the main loop, provide the current sensor reading and get the control output:
   ```cpp
   float output = pid.compute(sensorReading);
   ```
5. Use the output to control your actuator (e.g., heater, motor, LED, etc.).

---

## Examples

### **1. Temperature Control**

This example demonstrates how to use AutoTunePID to maintain a desired temperature using a temperature sensor and a heater/fan.

### **2. Distance Control**

This example shows how to maintain a target distance using an ultrasonic sensor and a motor.

### **3. Light Intensity Control**

This example explains how to regulate light intensity using a photodiode sensor and an LED.

Refer to the `examples` folder for complete sketches.

---

## Example Code

### **Basic Example**

```cpp
#include <AutoTunePID.h>

// Define parameters
#define SENSOR_PIN A0
#define OUTPUT_PIN 9

AutoTunePID pid(255, 0, 10000); // maxOutput, minOutput, tuningDuration

void setup() {
    pinMode(OUTPUT_PIN, OUTPUT);
    pid.setSetpoint(50.0); // Target value
    pid.startTuning();
}

void loop() {
    float sensorValue = analogRead(SENSOR_PIN);
    float output = pid.compute(sensorValue);
    analogWrite(OUTPUT_PIN, (int)output);
}
```

---

## Applications

- **Home Automation:** Temperature control for smart homes.
- **Robotics:** Distance maintenance for autonomous vehicles.
- **Energy Systems:** Light dimming and energy-efficient systems.
- **Industrial Control:** Process regulation for manufacturing systems.

---

## Contributing

Feel free to contribute to the library by submitting pull requests or reporting issues on the [GitHub repository](#).

---

## License

This library is licensed under the MIT License. See `LICENSE` for details.

