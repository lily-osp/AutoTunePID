# AutoTunePID Library

A robust, feature-rich PID control library for Arduino that implements advanced auto-tuning algorithms and signal filtering capabilities.

## Features

- **Advanced PID Control Implementation**
  - Real-time PID calculations with configurable update intervals
  - Constrained output range for system protection
  - Support for both auto-tuning and manual parameter configuration

- **Multiple Auto-Tuning Methods**
  - Ziegler-Nichols method
  - Cohen-Coon method
  - Manual tuning capability with direct gain settings

- **Signal Processing**
  - Configurable input signal filtering
  - Output smoothing capabilities
  - Exponential moving average filters with adjustable alpha values

- **Safety and Reliability**
  - Constrained output values
  - Fixed interval updates
  - Protected filter parameters

## Installation

1. Download the latest release from GitHub
2. In Arduino IDE: Sketch > Include Library > Add .ZIP Library
3. Select the downloaded file
4. Restart Arduino IDE

## Quick Start

```cpp
#include <AutoTunePID.h>

// Initialize PID controller with output range and tuning method
AutoTunePID pid(0, 255, ZieglerNichols);

void setup() {
    // Configure controller
    pid.setSetpoint(100.0);
    pid.enableInputFilter(0.1);  // Optional: Enable input filtering
}

void loop() {
    float input = analogRead(A0);
    pid.update(input);
    analogWrite(PWM_PIN, pid.getOutput());
}
```

## API Reference

### Core Functions

#### Initialization
```cpp
AutoTunePID(float minOutput, float maxOutput, TuningMethod method = ZieglerNichols);
```
- `minOutput`: Lower bound for controller output
- `maxOutput`: Upper bound for controller output
- `method`: Auto-tuning method selection

#### Control Configuration
```cpp
void setSetpoint(float setpoint);
void setTuningMethod(TuningMethod method);
void setManualGains(float kp, float ki, float kd);
```

#### Signal Filtering
```cpp
void enableInputFilter(float alpha);   // alpha range: 0.01-1.0
void enableOutputFilter(float alpha);  // alpha range: 0.01-1.0
```

#### Runtime Operations
```cpp
void update(float currentInput);  // Update at minimum 100ms intervals
float getOutput();
```

#### Parameter Access
```cpp
float getKp();
float getKi();
float getKd();
```

## Advanced Usage

### Auto-Tuning Process

The library implements two auto-tuning methods:

1. **Ziegler-Nichols Method**
   - Determines ultimate gain (Ku) and oscillation period (Tu)
   - Calculates parameters:
     - Kp = 0.6 * Ku
     - Ki = 2 * Kp / Tu
     - Kd = Kp * Tu / 8

2. **Cohen-Coon Method**
   - Uses similar principles but with different multipliers
   - Calculates parameters:
     - Kp = 1.35 * Ku
     - Ki = Kp / (2.5 * Tu)
     - Kd = 0.37 * Kp * Tu

### Signal Filtering

Input and output filters use an exponential moving average:
```cpp
filteredValue = (alpha * input) + ((1 - alpha) * filteredValue)
```
where `alpha` determines the filter's responsiveness (0.01-1.0)

## Application Examples

### Temperature Control System
```cpp
#include <AutoTunePID.h>

AutoTunePID tempController(0, 255, ZieglerNichols);

void setup() {
    tempController.setSetpoint(75.0);  // 75°C target
    tempController.enableInputFilter(0.1);  // Smooth temperature readings
    tempController.enableOutputFilter(0.2); // Smooth heater control
}

void loop() {
    float currentTemp = readTemperature();
    tempController.update(currentTemp);
    analogWrite(HEATER_PIN, tempController.getOutput());
    delay(100);
}
```

### Motor Speed Control
```cpp
#include <AutoTunePID.h>

AutoTunePID speedController(0, 255, CohenCoon);

void setup() {
    speedController.setSetpoint(1000);  // 1000 RPM target
    speedController.enableInputFilter(0.2);
}

void loop() {
    float currentSpeed = readEncoderSpeed();
    speedController.update(currentSpeed);
    analogWrite(MOTOR_PIN, speedController.getOutput());
    delay(100);
}
```

## Performance Considerations

- Update interval fixed at 100ms for stability
- Filter alpha values impact system responsiveness
- Auto-tuning duration affects parameter accuracy
- Memory usage: ~40 bytes for instance variables

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch
3. Submit a pull request

## License

This project is licensed under the MIT License. See LICENSE file for details.

## Support

For bug reports and feature requests, please use the GitHub issue tracker.

For technical questions, contact: azzar.mr.zs@gmail.com
