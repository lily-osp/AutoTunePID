# AutoTunePID Library

A robust, feature-rich PID control library for Arduino that implements advanced auto-tuning algorithms, signal filtering, and anti-windup capabilities.

---

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
  - **Tyreus-Luyben**: Provides robust tuning with minimal overshoot, ideal for systems requiring stability.

- **Signal Filtering**

  - Configurable input and output signal filters using exponential moving averages.
  - Adjustable alpha values for filter responsiveness (range: 0.01–1.0).

- **Anti-Windup**

  - Prevents integral windup by constraining the integral term when the output is saturated.
  - Configurable threshold for anti-windup behavior.

- **Safety and Reliability**

  - Output values are constrained within specified bounds.
  - Fixed interval updates ensure stability.
  - Protected filter parameters to prevent invalid configurations.

---

## Installation

1. Download the library as a ZIP file.
2. In the Arduino IDE, go to **Sketch > Include Library > Add .ZIP Library**.
3. Select the downloaded ZIP file.
4. Restart the Arduino IDE.

---

## Quick Start

```cpp
#include <AutoTunePID.h>

// Initialize PID controller with output range and tuning method
AutoTunePID pid(0, 255, TuningMethod::ZieglerNichols);

void setup() {
  pid.setSetpoint(100.0); // Target setpoint
  pid.enableInputFilter(0.1); // Optional input filtering
  pid.enableAntiWindup(true, 0.8); // Enable anti-windup with 80% threshold
}

void loop() {
  float input = analogRead(A0); // Read input
  pid.update(input); // Update the PID controller
  analogWrite(PWM_PIN, pid.getOutput()); // Write output
}
```

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

3. **Relay Feedback**:

   - Uses oscillations induced by a relay to compute parameters:
     - $ K_p = 0.5 \cdot Ku $
     - $ K_i = \frac{1.0 \cdot K_p}{Tu} $
     - $ K_d = 0.125 \cdot K_p \cdot Tu $

4. **IMC (Internal Model Control)**:

   - Incorporates a smoothing factor ('λ') to adjust response speed:
     - $ K_p = 0.4 \cdot Ku $
     - $ K_i = \frac{K_p}{2 \cdot \lambda} $
     - $ K_d = 0.5 \cdot K_p \cdot \lambda $

5. **Tyreus-Luyben**:

   - Provides robust tuning with minimal overshoot:
     - $ K_p = 0.45 \cdot Ku $
     - $ K_i = \frac{K_p}{2.2 \cdot Tu} $
     - $ K_d = 0.0 $ (No derivative term)

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
}

void loop() {
  float rpm = readEncoderSpeed(); // Read motor speed
  motorController.update(rpm); // Update PID controller
  analogWrite(MOTOR_PIN, motorController.getOutput()); // Control motor
  delay(100);
}
```

### 3. Relay Feedback Example: Water Level Control

```cpp
#include <AutoTunePID.h>
AutoTunePID waterController(0, 255, TuningMethod::RelayFeedback);

void setup() {
  waterController.setSetpoint(50.0); // Target water level
  waterController.enableInputFilter(0.15); // Enable input filtering
  waterController.enableAntiWindup(true, 0.9); // Enable anti-windup
}

void loop() {
  float level = readWaterLevel(); // Read water level sensor
  waterController.update(level); // Update PID controller
  analogWrite(VALVE_PIN, waterController.getOutput()); // Control valve
  delay(100);
}
```

### 4. IMC Example: Pressure Control

```cpp
#include <AutoTunePID.h>
AutoTunePID pressureController(0, 255, TuningMethod::IMC);

void setup() {
  pressureController.setSetpoint(100.0); // Target pressure
  pressureController.enableInputFilter(0.1); // Enable input filtering
  pressureController.enableAntiWindup(true, 0.8); // Enable anti-windup
}

void loop() {
  float pressure = readPressureSensor(); // Read pressure sensor
  pressureController.update(pressure); // Update PID controller
  analogWrite(PUMP_PIN, pressureController.getOutput()); // Control pump
  delay(100);
}
```

### 5. Tyreus-Luyben Example: Chemical Reactor Temperature Control

```cpp
#include <AutoTunePID.h>
AutoTunePID reactorController(0, 255, TuningMethod::TyreusLuyben);

void setup() {
  reactorController.setSetpoint(80.0); // Target reactor temperature
  reactorController.enableInputFilter(0.1); // Enable input filtering
  reactorController.enableAntiWindup(true, 0.8); // Enable anti-windup
}

void loop() {
  float temp = readReactorTemperature(); // Read reactor temperature
  reactorController.update(temp); // Update PID controller
  analogWrite(HEATER_PIN, reactorController.getOutput()); // Control heater
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

## Detailed Explanation

- For more details about the algorithms used in the library, read [here](docs/explanation.md).
- For more details about the library usage, read [here](docs/usage.md).
- For more details about manual tuning, read [here](docs/manual.md).

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

For bug reports and feature requests, please use the [GitHub issue tracker](https://github.com/your-repo/issues).

For technical questions, contact: azzar.mr.zs@gmail.com
