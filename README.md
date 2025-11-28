# AutoTunePID Library

A robust, feature-rich PID control library for Arduino that implements advanced auto-tuning algorithms, signal filtering, anti-windup, and operational modes for flexible control.

---

## Features

- **Comprehensive PID Control**
  
  - Real-time PID calculations with configurable update intervals.
  - Constrained output range to ensure system safety.
  - Support for both auto-tuning and manual parameter configuration.

- **Multiple Auto-Tuning Methods**:
  
  - **Ziegler-Nichols**: Determines ultimate gain (Ku) and oscillation period (Tu) using observed output extremes.
  - **Cohen-Coon**: Fine-tunes Ku and Tu with alternative multipliers for better initial performance.
  - **IMC (Internal Model Control)**: Balances system robustness and responsiveness using a lambda tuning parameter.
  - **Tyreus-Luyben**: Provides robust tuning with minimal overshoot, ideal for systems requiring stability.
  - **Lambda Tuning (CLD)**: Optimizes systems with significant dead time using process time constant (T) and dead time (L).

- **Operational Modes**:
  
  - **Normal**: Standard PID operation.
  - **Reverse**: Reverses the error calculation for cooling systems.
  - **Hold**: Stops all calculations to save resources.
  - **Preserve**: Minimal calculations to keep the system responsive.
  - **Tune**: Performs auto-tuning to determine `Tu` and `Ku`.
  - **Auto**: Automatically selects the best operational mode based on system behavior.

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
  pid.setOscillationMode(OscillationMode::Normal); // Set oscillation mode to Normal
  pid.setOperationalMode(OperationalMode::Tune); // Set operational mode to Tune
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

#### Operational Modes

```cpp
void setOperationalMode(OperationalMode mode); // Set the operational mode
```

#### Oscillation Modes

```cpp
void setOscillationMode(OscillationMode mode); // Set the oscillation mode for auto-tuning
void setOscillationSteps(int steps); // Set the number of oscillation steps for auto-tuning
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
     - $$
       K_p = 0.6 \cdot K_u
       $$
     - $$
       K_i = \frac{1.2 \cdot K_p}{T_u}
       $$
     - $$
       K_d = 0.075 \cdot K_p \cdot T_u
       $$

2. **Cohen-Coon**:
   
   - Alternative multipliers provide better transient response.
   - Gains are calculated as:
     - $$
       K_p = 0.8 \cdot K_u
       $$
     - $$
       K_i = \frac{K_p}{0.8 \cdot T_u}
       $$
     - $$
       K_d = 0.194 \cdot K_p \cdot T_u
       $$

3. **IMC (Internal Model Control)**:
   
   - Incorporates a smoothing factor ('λ') to adjust response speed:
     - $$
       K_p = 0.4 \cdot K_u
       $$
     - $$
       K_i = \frac{K_p}{2 \cdot \lambda}
       $$
     - $$
       K_d = 0.5 \cdot K_p \cdot \lambda
       $$

4. **Tyreus-Luyben**:
   
   - Provides robust tuning with minimal overshoot:
     - $$
       K_p = 0.45 \cdot K_u
       $$
     - $$
       K_i = \frac{K_p}{2.2 \cdot T_u}
       $$
     - $$
       K_d = 0.0
       $$ (No derivative term)

5. **Lambda Tuning (CLD)**:
   
   - Optimizes systems with significant dead time:
     - $$
       K_p = \frac{T}{K(\lambda + L)}
       $$
     - $$
       K_i = \frac{K_p}{T} = \frac{1}{K(\lambda + L)}
       $$
     - $$
       K_d = K_p \cdot 0.5L = \frac{0.5L \cdot T}{K(\lambda + L)}
       $$

---

## Signal Filtering

Filters smooth inputs and outputs using an exponential moving average:

- $$
  \text{filteredValue} = (\alpha \cdot \text{input}) + ((1 - \alpha) \cdot \text{filteredValue})
  $$
- $$ \alpha $$: Responsiveness of the filter (range: 0.01–1.0).

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
}

void loop() {
  float rpm = readEncoderSpeed(); // Read motor speed
  motorController.update(rpm); // Update PID controller
  analogWrite(MOTOR_PIN, motorController.getOutput()); // Control motor
  delay(100);
}
```

### 3. IMC Example: Pressure Control

```cpp
#include <AutoTunePID.h>
AutoTunePID pressureController(0, 255, TuningMethod::IMC);

void setup() {
  pressureController.setSetpoint(100.0); // Target pressure
  pressureController.enableInputFilter(0.1); // Enable input filtering
  pressureController.enableAntiWindup(true, 0.8); // Enable anti-windup
  pressureController.setOscillationMode(OscillationMode::Normal); // Set oscillation mode to Normal
  pressureController.setOperationalMode(OperationalMode::Tune); // Set operational mode to Tune
}

void loop() {
  float pressure = readPressureSensor(); // Read pressure sensor
  pressureController.update(pressure); // Update PID controller
  analogWrite(PUMP_PIN, pressureController.getOutput()); // Control pump
  delay(100);
}
```

### 4. Tyreus-Luyben Example: Chemical Reactor Temperature Control

```cpp
#include <AutoTunePID.h>
AutoTunePID reactorController(0, 255, TuningMethod::TyreusLuyben);

void setup() {
  reactorController.setSetpoint(80.0); // Target reactor temperature
  reactorController.enableInputFilter(0.1); // Enable input filtering
  reactorController.enableAntiWindup(true, 0.8); // Enable anti-windup
  reactorController.setOscillationMode(OscillationMode::Half); // Set oscillation mode to Half
  reactorController.setOperationalMode(OperationalMode::Tune); // Set operational mode to Tune
}

void loop() {
  float temp = readReactorTemperature(); // Read reactor temperature
  reactorController.update(temp); // Update PID controller
  analogWrite(HEATER_PIN, reactorController.getOutput()); // Control heater
  delay(100);
}
```

### 5. Lambda Tuning Example: Flow Control

```cpp
#include <AutoTunePID.h>
AutoTunePID flowController(0, 255, TuningMethod::LambdaTuning);

void setup() {
  flowController.setSetpoint(50.0); // Target flow rate
  flowController.enableInputFilter(0.15); // Enable input filtering
  flowController.enableAntiWindup(true, 0.9); // Enable anti-windup
  flowController.setOscillationMode(OscillationMode::Mild); // Set oscillation mode to Mild
  flowController.setOperationalMode(OperationalMode::Tune); // Set operational mode to Tune
}

void loop() {
  float flowRate = readFlowSensor(); // Read flow sensor
  flowController.update(flowRate); // Update PID controller
  analogWrite(VALVE_PIN, flowController.getOutput()); // Control valve
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
- **Getting Started**: New to PID control? Start [here](docs/getting_started.md)
- **API Reference**: Complete method reference with examples [here](docs/api_reference.md)
- **Usage Guide**: Detailed usage instructions [here](docs/usage.md)
- **Integration Guide**: Connect with sensors/actuators/libraries [here](docs/integration_guide.md)
- **Best Practices**: Optimization and design guidelines [here](docs/best_practices.md)
- **Troubleshooting**: Common issues and solutions [here](docs/troubleshooting.md)
- **Manual Tuning**: Step-by-step tuning procedures [here](docs/manual.md)
- **System Architecture**: Internal design overview [here](docs/system_architecture.md)
- **Algorithm Details**: Mathematical formulas and explanations [here](docs/formula.md)
- **Code Examples**: Practical implementations [here](docs/examples.md)
- **Visual Diagrams**:
  - [Flowchart](docs/flowchart.mermaid)
  - [State diagram](docs/state-diagram.mermaid)

---

## Releases and Installation

### Automated Releases

This repository uses GitHub Actions to automatically create releases with Arduino library ZIP files:

#### Automatic Release Creation
- **Tag-based releases**: Push a version tag (e.g., `v1.2.3`) to automatically create a release
- **Manual releases**: Use the "Manual Release Creation" workflow in GitHub Actions
- **Release validation**: All releases are validated for proper Arduino library structure

#### Release Assets
Each release includes:
- **`AutoTunePID-x.x.x.zip`**: Arduino library ZIP file for IDE installation
- Complete library structure optimized for Arduino IDE Library Manager

### Installation Methods

#### Method 1: Arduino IDE (Recommended)
1. Go to [Releases](https://github.com/lily-osp/AutoTunePID/releases)
2. Download the latest `AutoTunePID-x.x.x.zip` file
3. Open Arduino IDE
4. Go to **Sketch > Include Library > Add .ZIP Library**
5. Select the downloaded ZIP file
6. Restart Arduino IDE

#### Method 2: Manual Installation
1. Download and extract the ZIP file
2. Copy the `AutoTunePID` folder to your Arduino libraries directory:
   - Windows: `Documents\Arduino\libraries\`
   - macOS: `~/Documents/Arduino/libraries/`
   - Linux: `~/Arduino/libraries/`
3. Restart Arduino IDE

#### Method 3: Arduino CLI
```bash
arduino-cli lib install --zip-path AutoTunePID-x.x.x.zip
```

### Development Workflow

#### For Contributors
1. **Validation**: All pushes are automatically validated for proper library structure
2. **Testing**: Examples are compiled to ensure functionality
3. **Releases**: Use GitHub Actions workflows for consistent releases

#### Creating a New Release
1. Update `version` in `library.properties`
2. Commit the change
3. Create and push a version tag: `git tag v1.2.3 && git push origin v1.2.3`
4. GitHub Actions will automatically create the release with ZIP file

#### Local Testing
Before releasing, test the ZIP creation locally:
```bash
./create_release_zip.sh
```
This creates the same ZIP structure that GitHub Actions will generate.

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
