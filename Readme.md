# AutoTunePID Library

AutoTunePID is an advanced PID (Proportional-Integral-Derivative) control library designed for Arduino. It provides robust auto-tuning mechanisms with support for multiple tuning methods, input/output filtering, and manual gain adjustments. This library is ideal for projects requiring precise and stable control over mechanical, electrical, or software systems.

---

## Features

- **Automatic Tuning**:
  - Implements Ziegler-Nichols and Cohen-Coon tuning methods.
- **Manual Tuning**:
  - Allows users to set custom PID gains.
- **Input and Output Filtering**:
  - Smooths noisy signals using configurable low-pass filters.
- **Flexible Update Intervals**:
  - Ensures calculations occur at consistent time steps.
- **Customizable Tuning Duration**:
  - Allows fine control over tuning period.

---

## Installation

1. Download the library files and place them in the `libraries` folder of your Arduino IDE directory.
2. Ensure the structure looks like this:

```
/libraries
  /AutoTunePID
    AutoTunePID.h
    AutoTunePID.cpp
    examples
      BasicPID
      InputFilter
      OutputFilter
```

3. Restart your Arduino IDE.

---

## Usage

### Initialization

Include the library in your Arduino sketch:
```cpp
#include <AutoTunePID.h>
```

Create an instance of the `AutoTunePID` class:
```cpp
AutoTunePID pidController(minOutput, maxOutput, tuningMethod);
```
- `minOutput`: Minimum output value (e.g., 0).
- `maxOutput`: Maximum output value (e.g., 255).
- `tuningMethod`: Choose from `ZieglerNichols`, `CohenCoon`, or `Manual`.

---

### Setting Parameters

#### Setpoint
Define the target value the PID controller should achieve:
```cpp
pidController.setSetpoint(50.0); // Example: Target temperature, position, etc.
```

#### Manual Gains
Set custom gains for `Kp`, `Ki`, and `Kd`:
```cpp
pidController.setManualGains(1.0, 0.5, 0.1);
```

#### Enable Filters
Enable input and output filtering for smoother operation:
```cpp
pidController.enableInputFilter(0.2);  // Alpha = 0.2 for input filtering
pidController.enableOutputFilter(0.1); // Alpha = 0.1 for output filtering
```

---

### Updating and Retrieving Output

Update the PID controller with the current input value and retrieve the output:
```cpp
pidController.update(currentInput);
float output = pidController.getOutput();
```

Apply the output to your system (e.g., motor speed, heater power).

---

## Examples

The library includes three example sketches to demonstrate various features:

### Example 1: Basic PID
Demonstrates basic PID control with automatic tuning (Ziegler-Nichols or Cohen-Coon) and manual tuning options.

### Example 2: Input Filtering
Shows how to enable input filtering to smooth noisy sensor data before PID calculation.

### Example 3: Output Filtering
Illustrates the use of output filtering to stabilize actuator signals and reduce abrupt changes.

---

## API Reference

### Constructor
```cpp
AutoTunePID(float minOutput, float maxOutput, TuningMethod method = ZieglerNichols);
```

### Methods

- **Setpoint and Gains**:
  - `void setSetpoint(float setpoint);`
  - `void setManualGains(float kp, float ki, float kd);`

- **Tuning**:
  - `void setTuningMethod(TuningMethod method);`
  - `float getKp();`
  - `float getKi();`
  - `float getKd();`

- **Input/Output Filtering**:
  - `void enableInputFilter(float alpha);`
  - `void enableOutputFilter(float alpha);`

- **Update and Retrieve Output**:
  - `void update(float currentInput);`
  - `float getOutput();`

---

## Example Code

### Basic PID Example
```cpp
#include <AutoTunePID.h>

AutoTunePID pidController(0, 255, ZieglerNichols);

void setup() {
  Serial.begin(9600);
  pidController.setSetpoint(50.0); // Example setpoint
}

void loop() {
  float sensorValue = analogRead(A0);
  pidController.update(sensorValue);
  float output = pidController.getOutput();
  analogWrite(9, output);
}
```

---

## Notes

- Use appropriate filter alpha values (0.01–1.0) to balance responsiveness and smoothness.
- Ensure the tuning duration matches the dynamics of your system for accurate auto-tuning.
- Always test the PID controller in a controlled environment before deploying it in real-world applications.

---

## License

This library is licensed under the MIT License. Feel free to use, modify, and distribute it for personal and commercial projects.

---

For further assistance or to report issues, please contact [support@example.com].

