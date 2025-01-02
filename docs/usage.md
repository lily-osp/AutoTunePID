# Using the AutoTunePID Library

The `AutoTunePID` library is a powerful tool for adaptive PID control in Arduino projects. It features automatic tuning based on methods like Ziegler-Nichols or Cohen-Coon, as well as manual tuning options. This guide provides a detailed explanation of how to integrate and use the library effectively.

## Initialization

To initialize the `AutoTunePID` controller, you need to specify the minimum and maximum output values for the PID controller. Additionally, you can define the tuning method, which defaults to `ZieglerNichols`.

### Example
```cpp
#include "AutoTunePID.h"

// Create an instance of AutoTunePID with a specified output range
AutoTunePID pidController(-255, 255, ZieglerNichols);
```

In this example, the `pidController` is configured to output values between -255 and 255 using the Ziegler-Nichols tuning method.

## Setting the Setpoint

The setpoint represents the target value that the system aims to achieve. Use `setSetpoint()` to define it.

### Example
```cpp
pidController.setSetpoint(100.0); // Set the target value to 100
```

This sets the desired system state to a value of 100.

## Selecting the Tuning Method

The tuning method determines how the PID gains are calculated. Use `setTuningMethod()` to choose one of the following options:

- `ZieglerNichols`: A popular method for process control.
- `CohenCoon`: Useful for processes with significant time delays.
- `Manual`: For direct user-defined gains.

### Example
```cpp
pidController.setTuningMethod(CohenCoon);
```

This sets the tuning method to Cohen-Coon for better performance in specific systems.

## Manual Tuning

If you prefer manual tuning, set the PID gains directly using `setManualGains()`.

### Example
```cpp
pidController.setManualGains(1.0, 0.5, 0.1); // Set Kp, Ki, and Kd
```

This allows precise control over the proportional, integral, and derivative gains.

## Input and Output Filtering

Input and output filtering smoothens noisy signals, enhancing the controller's performance. Enable filtering and define a smoothing factor (alpha) between 0.01 and 1.0. Smaller values result in more smoothing.

- `enableInputFilter(alpha)`: Smooths the input signal.
- `enableOutputFilter(alpha)`: Smooths the output signal.

### Example
```cpp
pidController.enableInputFilter(0.2); // Apply input smoothing with alpha = 0.2
pidController.enableOutputFilter(0.3); // Apply output smoothing with alpha = 0.3
```

These functions improve stability in systems prone to noise.

## Updating the Controller

The `update()` function processes the current input and calculates the appropriate output. Call it within the control loop.

### Example
```cpp
void loop() {
    float sensorValue = analogRead(A0); // Read sensor input
    pidController.update(sensorValue);

    float output = pidController.getOutput();
    analogWrite(3, output); // Send output to the actuator
}
```

This example continuously updates the PID output based on the sensor reading.

## Retrieving PID Gains and Output

Retrieve the computed or manually set PID gains and the current output value:

- `getKp()`, `getKi()`, `getKd()`: Access the proportional, integral, and derivative gains.
- `getOutput()`: Access the controller's current output.

### Example
```cpp
float kp = pidController.getKp();
float ki = pidController.getKi();
float kd = pidController.getKd();

Serial.print("Kp: "); Serial.println(kp);
Serial.print("Ki: "); Serial.println(ki);
Serial.print("Kd: "); Serial.println(kd);
```

This prints the current PID parameters to the Serial Monitor.

## Auto-Tuning Behavior

When auto-tuning is enabled, the library uses either Ziegler-Nichols or Cohen-Coon methods to compute optimal gains over a specified tuning duration (default: 5000 ms). These gains are applied automatically after tuning completes.

### Notes
- Ensure the system can oscillate safely during the tuning process.
- Adjust tuning duration and output limits to match your system's dynamics.

## Example Sketch

This sketch demonstrates the full functionality of the `AutoTunePID` library:

```cpp
#include "AutoTunePID.h"

AutoTunePID pidController(-255, 255);

void setup() {
    Serial.begin(9600);
    pidController.setSetpoint(50.0); // Set the target value
    pidController.enableInputFilter(0.1); // Smooth input signals
    pidController.enableOutputFilter(0.1); // Smooth output signals
}

void loop() {
    float currentInput = analogRead(A0) * (5.0 / 1023.0); // Convert to voltage
    pidController.update(currentInput);

    float output = pidController.getOutput();
    analogWrite(3, output); // Control actuator

    Serial.print("Output: "); Serial.println(output);
}
```

This sketch integrates all key features of the library, showcasing its practical application.

## Summary of Methods

| Method                              | Description                                   |
|-------------------------------------|-----------------------------------------------|
| `setSetpoint(float setpoint)`       | Sets the target value for the PID controller.|
| `setTuningMethod(TuningMethod)`     | Selects the tuning method.                   |
| `setManualGains(float kp, ki, kd)`  | Sets PID gains manually.                     |
| `enableInputFilter(float alpha)`    | Enables input filtering with a smoothing factor. |
| `enableOutputFilter(float alpha)`   | Enables output filtering with a smoothing factor. |
| `update(float currentInput)`        | Updates the PID controller with the current input. |
| `getOutput()`                       | Retrieves the computed PID output.           |
| `getKp()`, `getKi()`, `getKd()`     | Retrieves the PID gains.                     |

Use this guide to take full advantage of the `AutoTunePID` library in your Arduino projects.

