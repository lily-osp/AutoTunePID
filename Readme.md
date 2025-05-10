# AutoTunePID Library

A robust, feature-rich PID control library for Arduino that implements advanced auto-tuning algorithms, signal filtering, anti-windup, and operational modes for flexible control.

---

## Features

- **Comprehensive PID Control**:
  - Real-time PID calculations with configurable update intervals.
  - Constrained output range to ensure system safety.
  - Support for both auto-tuning and manual parameter configuration.

- **Multiple Auto-Tuning Methods**:
  - **Ziegler-Nichols**: Determines ultimate gain (Ku) and oscillation period (Tu) using observed output extremes.
  - **Cohen-Coon**: Fine-tunes Ku and Tu with alternative multipliers for better initial performance.
  - **IMC (Internal Model Control)**: Balances system robustness and responsiveness using a lambda tuning parameter.
  - **Tyreus-Luyben**: Provides robust tuning with minimal overshoot, ideal for systems requiring stability.
  - **Lambda Tuning (CLD)**: Optimizes systems with significant dead time using process time constant (T) and dead time (L).

- **System Corrector**:
  - Monitors system response for instability using historical data points.
  - Applies corrective actions (e.g., reducing output, resetting integral term).
  - Optionally re-tunes the PID gains if instability persists.

- **Operational Modes**:
  - **Normal**: Standard PID operation.
  - **Reverse**: Reverses the error calculation for cooling systems.
  - **Hold**: Stops all calculations to save resources.
  - **Preserve**: Minimal calculations to keep the system responsive.
  - **Tune**: Performs auto-tuning to determine `Tu` and `Ku`.
  - **Auto**: Automatically determines the best operational mode based on system behavior.

- **Oscillation Modes**:
  - **Normal**: Full oscillation (`MaxOutput - MinOutput`).
  - **Half**: Half oscillation (`1/2 MaxOutput - 1/2 MinOutput`).
  - **Mild**: Mild oscillation (`1/4 MaxOutput - 1/4 MinOutput`).

- **Signal Filtering**:
  - Configurable input and output signal filters using exponential moving averages.
  - Adjustable alpha values for filter responsiveness (range: 0.01–1.0).

- **Anti-Windup**:
  - Prevents integral windup by constraining the integral term when the output is saturated.
  - Configurable threshold for anti-windup behavior.

- **Safety and Reliability**:
  - Output values are constrained within specified bounds.
  - Fixed interval updates ensure stability.
  - Protected filter parameters to prevent invalid configurations.

---

## Getting Started

### Installation
1. Copy the `AutoTunePID` library folder into your Arduino `libraries` directory.
2. Restart the Arduino IDE.
3. Open **File > Examples > AutoTunePID > BasicUsage** to get started.

### Basic Example
See [`examples/BasicUsage/BasicUsage.ino`](examples/BasicUsage/BasicUsage.ino) for a minimal usage example.

```cpp
#include <AutoTunePID.h>

AutoTunePID pid(0, 255, TuningMethod::ZieglerNichols);

void setup() {
  Serial.begin(9600);
  pid.setSetpoint(100); // Target value
}

void loop() {
  float input = analogRead(A0); // Simulated process variable
  pid.update(input);
  float output = pid.getOutput();
  analogWrite(3, output); // Simulated actuator
  Serial.print("Input: "); Serial.print(input);
  Serial.print(" Output: "); Serial.println(output);
  delay(100);
}
```

## Tuning Methods
- **Ziegler-Nichols:** Classic method for aggressive tuning, good for fast response but may overshoot.
- **Cohen-Coon:** Better for processes with significant dead time.
- **IMC (Internal Model Control):** Provides a balance between performance and robustness.
- **Tyreus-Luyben:** More conservative than Ziegler-Nichols, less overshoot.
- **Lambda Tuning (CLD):** Good for setpoint tracking and disturbance rejection.
- **Manual:** User sets Kp, Ki, Kd directly.

See [docs/2. Formula.md](docs/2. Formula.md) for detailed formulas and [docs/3. Usage.md](docs/3. Usage.md) for advanced usage.

## Troubleshooting / FAQ
- **Q: My output is always zero or maxed out.**
  - A: Check your input range and setpoint. Ensure the PID gains are reasonable.
- **Q: Auto-tuning never finishes.**
  - A: Make sure your process can oscillate and the system is not too slow or too fast for the default settings.
- **Q: How do I use a different tuning method?**
  - A: Use `pid.setTuningMethod(TuningMethod::IMC);` or another method before calling `update()`.
- **Q: Can I use this with non-Arduino platforms?**
  - A: The core logic is standard C++, but timing and IO are Arduino-specific by default.

## More Documentation
- See the `docs/` folder for detailed explanations, hardware design, and configuration options.
- Each example in `examples/` has its own README for setup and expected results.

---

## API Reference

### Core Functions

#### Initialization

```cpp
AutoTunePID(float minOutput, float maxOutput, TuningMethod method = TuningMethod::ZieglerNichols);
```

- `minOutput`: Lower bound for controller output.
- `maxOutput`: Upper bound for controller output.
- `method`: Auto-tuning method selection (default: Ziegler-Nichols).

#### Control Configuration

```cpp
void setSetpoint(float setpoint); // Set the desired setpoint
void setTuningMethod(TuningMethod method); // Change the tuning method
void setManualGains(float kp, float ki, float kd); // Set manual PID gains
```

#### Signal Filtering

```cpp
void enableInputFilter(float alpha);  // Enable input filtering (alpha range: 0.01-1.0)
void enableOutputFilter(float alpha); // Enable output filtering (alpha range: 0.01-1.0)
```

#### Anti-Windup

```cpp
void enableAntiWindup(bool enable, float threshold = 0.8f); // Enable/disable anti-windup with optional threshold
```

#### Operational Modes

```cpp
void setOperationalMode(OperationalMode mode); // Set the operational mode
```

#### Oscillation Modes

```cpp
void setOscillationMode(OscillationMode mode); // Set the oscillation mode for auto-tuning
void setOscillationSteps(int steps); // Set the number of oscillation steps for auto-tuning
```

#### Corrector

```cpp
void enableCorrector(bool enable, int dataWindowSize = 10, float stabilityThreshold = 0.1f); // Enable/disable corrector
```

#### Runtime Operations

```cpp
void update(float currentInput); // Update at minimum 100ms intervals
float getOutput(); // Get the current output value
```

#### Parameter Access

```cpp
float getKp(); // Get the proportional gain (Kp)
float getKi(); // Get the integral gain (Ki)
float getKd(); // Get the derivative gain (Kd)
float getKu(); // Get the ultimate gain (Ku)
float getTu(); // Get the oscillation period (Tu)
float getSetpoint(); // Get the current setpoint
```

---

## Advanced Auto-Tuning Methods

The library implements five distinct auto-tuning algorithms:

1. **Ziegler-Nichols**:
   - Oscillates the system to determine Ku and Tu based on output extremes.
   - Calculates PID gains:
     - $ K_p = 0.6 \cdot Ku $
     - $ K_i = \frac{1.2 \cdot K_p}{Tu} $
     - $ K_d = 0.075 \cdot K_p \cdot Tu $

2. **Cohen-Coon**:
   - Alternative multipliers provide better transient response.
   - Gains are calculated as:
     - $ K_p = 0.8 \cdot Ku $
     - $ K_i = \frac{K_p}{0.8 \cdot Tu} $
     - $ K_d = 0.194 \cdot K_p \cdot Tu $

3. **IMC (Internal Model Control)**:
   - Incorporates a smoothing factor ('λ') to adjust response speed:
     - $ K_p = 0.4 \cdot Ku $
     - $ K_i = \frac{K_p}{2 \cdot \lambda} $
     - $ K_d = 0.5 \cdot K_p \cdot \lambda $

4. **Tyreus-Luyben**:
   - Provides robust tuning with minimal overshoot:
     - $ K_p = 0.45 \cdot Ku $
     - $ K_i = \frac{K_p}{2.2 \cdot Tu} $
     - $ K_d = 0.0 $ (No derivative term)

5. **Lambda Tuning (CLD)**:
   - Optimizes systems with significant dead time:
     - $ K_p = \frac{T}{K(\lambda + L)} $
     - $ K_i = \frac{K_p}{T} = \frac{1}{K(\lambda + L)} $
     - $ K_d = K_p \cdot 0.5L = \frac{0.5L \cdot T}{K(\lambda + L)} $

---

## Signal Filtering

Filters smooth inputs and outputs using an exponential moving average:

- $\text{filteredValue} = (\alpha \cdot \text{input}) + ((1 - \alpha) \cdot \text{filteredValue})$
- $ \alpha $: Responsiveness of the filter (range: 0.01–1.0).

---

## Example Applications

### 1. Ziegler-Nichols Example: Temperature Control

```cpp
#include <AutoTunePID.h>
AutoTunePID tempController(0, 255, TuningMethod::ZieglerNichols);

void setup() {
  tempController.setSetpoint(75.0); // Target temperature
  tempController.enableInputFilter(0.1); // Enable input filtering
  tempController.enableAntiWindup(true, 0.8); // Enable anti-windup
  tempController.setOscillationMode(OscillationMode::Normal); // Set oscillation mode to Normal
  tempController.setOperationalMode(OperationalMode::Tune); // Set operational mode to Tune
  tempController.enableCorrector(true, 10, 5.0); // Enable corrector
}

void loop() {
  float temp = readTemperature(); // Read temperature sensor
  tempController.update(temp); // Update PID controller
  analogWrite(HEATER_PIN, tempController.getOutput()); // Control heater
  delay(100);
}
```

### 2. Cohen-Coon Example: Motor Speed Control

```cpp
#include <AutoTunePID.h>
AutoTunePID motorController(0, 255, TuningMethod::CohenCoon);

void setup() {
  motorController.setSetpoint(1500); // Target RPM
  motorController.enableInputFilter(0.2); // Enable input filtering
  motorController.enableAntiWindup(true, 0.7); // Enable anti-windup
  motorController.setOscillationMode(OscillationMode::Half); // Set oscillation mode to Half
  motorController.setOperationalMode(OperationalMode::Tune); // Set operational mode to Tune
  motorController.enableCorrector(true, 10, 5.0); // Enable corrector
}

void loop() {
  float rpm = readEncoderSpeed(); // Read motor speed
  motorController.update(rpm); // Update PID controller
  analogWrite(MOTOR_PIN, motorController.getOutput()); // Control motor
  delay(100);
}
```

---

## Performance Considerations

- Update interval fixed at 100ms for stability.
- Filter alpha values impact system responsiveness.
- Auto-tuning duration configurable (default: 5 seconds).
- Memory footprint optimized (~40 bytes per instance).

---

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository.
2. Create a feature branch.
3. Submit a pull request.

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) file for details.

---

## Support

For bug reports and feature requests, please use the [GitHub issue tracker](https://github.com/lily-osp/AutoTunePID/issues).

For technical questions, contact: azzar.mr.zs@gmail.com

---
