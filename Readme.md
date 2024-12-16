# AutoTunePID Library

A robust, feature-rich PID control library for Arduino that implements advanced auto-tuning algorithms and signal filtering capabilities.

## Features

- **Comprehensive PID Control**
  - Real-time PID calculations with configurable update intervals.
  - Constrained output range to ensure system safety.
  - Support for both auto-tuning and manual parameter configuration.

- **Multiple Auto-Tuning Methods**:
  - **Ziegler-Nichols**: Determines ultimate gain (Ku) and oscillation period (Tu) using observed output extremes.
  - **Cohen-Coon**: Fine-tunes Ku and Tu with alternative multipliers for better initial performance.
  - **Relay Feedback**: Identifies Ku and Tu via relay oscillations, suited for systems without steady-state errors.
  - **IMC (Internal Model Control)**: Balances system robustness and responsiveness using a lambda tuning parameter.

- **Signal Filtering**
  - Configurable input and output signal filters using exponential moving averages.
  - Adjustable alpha values for filter responsiveness (range: 0.01–1.0).

- **Safety and Reliability**
  - Output values are constrained within specified bounds.
  - Fixed interval updates ensure stability.
  - Protected filter parameters to prevent invalid configurations.

## Installation

1. Download the library as a ZIP file
2. In the Arduino IDE, go to Sketch > Include Library > Add .ZIP Library
3. Select the downloaded ZIP file
4. Restart the Arduino IDE

## Quick Start

```cpp
#include <AutoTunePID.h>

// Initialize PID controller with output range and tuning method
AutoTunePID pid(0, 255, ZieglerNichols);

void setup() {
    pid.setSetpoint(100.0); // Target setpoint
    pid.enableInputFilter(0.1); // Optional input filtering
}

void loop() {
    float input = analogRead(A0); // Read input
    pid.update(input); // Update the PID controller
    analogWrite(PWM_PIN, pid.getOutput()); // Write output
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

## Advanced Auto-Tuning Methods

The library implements four distinct auto-tuning algorithms:

1. **Ziegler-Nichols**:
   - Oscillates the system to determine Ku and Tu based on output extremes.
   - Calculates PID gains:
     - $`\( K_p = 0.6 \cdot Ku \)`$
     - $`\( K_i = \frac{2 \cdot K_p}{Tu} \)`$
     - $\( K_d = \frac{K_p \cdot Tu}{8} \)$

2. **Cohen-Coon**:
   - Alternative multipliers provide better transient response.
   - Gains are calculated as:
     - \( K_p = 1.35 \cdot Ku \)
     - \( K_i = \frac{K_p}{2.5 \cdot Tu} \)
     - \( K_d = 0.37 \cdot K_p \cdot Tu \)

3. **Relay Feedback**:
   - Uses oscillations induced by a relay to compute parameters:
     - \( K_p = 0.6 \cdot Ku \)
     - \( K_i = \frac{1.2 \cdot K_p}{Tu} \)
     - \( K_d = 0.075 \cdot K_p \cdot Tu \)

4. **IMC (Internal Model Control)**:
   - Incorporates a smoothing factor (\( \lambda \)) to adjust response speed:
     - \( K_p = \frac{Ku}{\lambda + Tu} \)
     - \( K_i = \frac{K_p}{\lambda + Tu} \)
     - \( K_d = K_p \cdot \frac{Tu \cdot \lambda}{\lambda + Tu} \)

## Signal Filtering

Filters smooth inputs and outputs using an exponential moving average:
\[ \text{filteredValue} = (\alpha \cdot \text{input}) + ((1 - \alpha) \cdot \text{filteredValue}) \]
- \( \alpha \): Responsiveness of the filter (range: 0.01–1.0).

## Example Applications

### Temperature Control
```cpp
#include <AutoTunePID.h>
AutoTunePID tempController(0, 255, IMC);

void setup() {
    tempController.setSetpoint(75.0);  // Target temperature
    tempController.enableInputFilter(0.1);
    tempController.enableOutputFilter(0.2);
}

void loop() {
    float temp = readTemperature();
    tempController.update(temp);
    analogWrite(HEATER_PIN, tempController.getOutput());
    delay(100);
}
```

### Motor Speed Control
```cpp
#include <AutoTunePID.h>
AutoTunePID motorController(0, 255, CohenCoon);

void setup() {
    motorController.setSetpoint(1500); // Target RPM
    motorController.enableInputFilter(0.2);
}

void loop() {
    float rpm = readEncoderSpeed();
    motorController.update(rpm);
    analogWrite(MOTOR_PIN, motorController.getOutput());
    delay(100);
}
```

## Performance Considerations

- Update interval fixed at 100ms for stability
- Filter alpha values impact system responsiveness
- Auto-tuning duration configurable (default: 5 seconds).
- Memory footprint optimized (~40 bytes per instance).

## Detailed explanation

- for more detailed about the algorithms used on the library you can read [here](info/explanation.md)
- for more detailed about the library usage you can read [here](info/usage.md)
- for more detailed about the how to do manual tuning you can read [here](info/manual.md)

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
