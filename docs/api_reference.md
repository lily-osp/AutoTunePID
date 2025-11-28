# AutoTunePID API Reference

This document provides a complete reference for all AutoTunePID library methods, enumerations, and configuration options.

## Table of Contents

- [Constructor](#constructor)
- [Configuration Methods](#configuration-methods)
- [Runtime Methods](#runtime-methods)
- [Operational Modes](#operational-modes)
- [Tuning Methods](#tuning-methods)
- [Oscillation Modes](#oscillation-modes)
- [Enumerations](#enumerations)
- [Error Codes](#error-codes)

## Constructor

### AutoTunePID(float minOutput, float maxOutput, TuningMethod method = TuningMethod::ZieglerNichols)

Creates a new AutoTunePID controller instance.

**Parameters:**
- `minOutput` (float): Minimum output value (e.g., 0 for PWM)
- `maxOutput` (float): Maximum output value (e.g., 255 for PWM)
- `method` (TuningMethod): Auto-tuning algorithm (default: ZieglerNichols)

**Example:**
```cpp
// PWM control (0-255)
AutoTunePID pid(0, 255, TuningMethod::ZieglerNichols);

// Servo control (0-180 degrees)
AutoTunePID servoPID(0, 180, TuningMethod::CohenCoon);
```

## Configuration Methods

### void setSetpoint(float setpoint)

Sets the target value for the control loop.

**Parameters:**
- `setpoint` (float): Desired target value

**Example:**
```cpp
pid.setSetpoint(25.0); // Target temperature 25°C
```

### void setTuningMethod(TuningMethod method)

Changes the auto-tuning algorithm.

**Parameters:**
- `method` (TuningMethod): Tuning algorithm to use

**Example:**
```cpp
pid.setTuningMethod(TuningMethod::IMC);
```

### void setManualGains(float kp, float ki, float kd)

Manually sets PID gain values (bypasses auto-tuning).

**Parameters:**
- `kp` (float): Proportional gain
- `ki` (float): Integral gain
- `kd` (float): Derivative gain

**Example:**
```cpp
pid.setManualGains(2.0, 0.5, 0.1);
```

### void enableInputFilter(float alpha)

Enables exponential moving average filtering on input signal.

**Parameters:**
- `alpha` (float): Filter coefficient (0.01-1.0, higher = more responsive)

**Example:**
```cpp
pid.enableInputFilter(0.1); // Smooth input signal
```

### void enableOutputFilter(float alpha)

Enables exponential moving average filtering on output signal.

**Parameters:**
- `alpha` (float): Filter coefficient (0.01-1.0, higher = more responsive)

**Example:**
```cpp
pid.enableOutputFilter(0.2); // Smooth output signal
```

### void enableAntiWindup(bool enable, float threshold = 0.8f)

Enables or disables integral windup protection.

**Parameters:**
- `enable` (bool): Enable/disable anti-windup
- `threshold` (float): Windup threshold (0.0-1.0, default: 0.8)

**Example:**
```cpp
pid.enableAntiWindup(true, 0.9); // Enable with 90% threshold
```

### void setOperationalMode(OperationalMode mode)

Sets the operational mode for the controller.

**Parameters:**
- `mode` (OperationalMode): Operational mode

**Example:**
```cpp
pid.setOperationalMode(OperationalMode::Reverse); // For cooling systems
```

### void setOscillationMode(OscillationMode mode)

Sets the oscillation mode for auto-tuning.

**Parameters:**
- `mode` (OscillationMode): Oscillation mode

**Example:**
```cpp
pid.setOscillationMode(OscillationMode::Half); // Reduced oscillation
```

### void setManualOutput(float output)

Sets output value for Manual mode (0-100% range).

**Parameters:**
- `output` (float): Output percentage (0-100)

**Example:**
```cpp
pid.setOperationalMode(OperationalMode::Manual);
pid.setManualOutput(75.0); // 75% output
```

### void setOverrideOutput(float output)

Sets fixed output value for Override mode.

**Parameters:**
- `output` (float): Fixed output value (constrained to min/max)

**Example:**
```cpp
pid.setOperationalMode(OperationalMode::Override);
pid.setOverrideOutput(255.0); // Maximum output
```

### void setTrackReference(float reference)

Sets reference signal for Track mode.

**Parameters:**
- `reference` (float): Reference value to track

**Example:**
```cpp
pid.setOperationalMode(OperationalMode::Track);
pid.setTrackReference(150.0); // Track this value
```

## Runtime Methods

### void update(float currentInput)

Updates the PID controller with current process variable.

**Parameters:**
- `currentInput` (float): Current process variable value

**Example:**
```cpp
float temperature = readTemperature();
pid.update(temperature);
```

### float getOutput()

Returns the current control output value.

**Returns:** (float) Current output value

**Example:**
```cpp
float output = pid.getOutput();
analogWrite(pin, output);
```

### float getKp(), getKi(), getKd()

Returns current PID gain values.

**Returns:** (float) Current gain value

**Example:**
```cpp
Serial.print("Kp: ");
Serial.println(pid.getKp());
```

### float getKu(), getTu()

Returns ultimate gain and oscillation period from auto-tuning.

**Returns:** (float) Auto-tuning results

**Example:**
```cpp
if (pid.getOperationalMode() == OperationalMode::Tune) {
    float ku = pid.getKu();
    float tu = pid.getTu();
}
```

### float getSetpoint()

Returns current setpoint value.

**Returns:** (float) Current setpoint

**Example:**
```cpp
float target = pid.getSetpoint();
```

### OperationalMode getOperationalMode()

Returns current operational mode.

**Returns:** (OperationalMode) Current mode

**Example:**
```cpp
if (pid.getOperationalMode() == OperationalMode::Normal) {
    // PID control active
}
```

## Operational Modes

| Mode | Description | Error Calculation | Use Case |
|------|-------------|-------------------|----------|
| **Normal** | Standard PID control | `setpoint - input` | Heating systems, most applications |
| **Reverse** | Reverse PID control | `input - setpoint` | Cooling systems, Peltier cells |
| **Manual** | Direct output control | None | Testing, calibration, manual override |
| **Override** | Fixed output value | None | Emergency conditions, safety systems |
| **Track** | Follow reference signal | None | Startup sequences, bumpless transfer |
| **Hold** | Freeze current output | None | Pause control, resource saving |
| **Preserve** | Minimal calculations | Basic monitoring | Standby with responsiveness |
| **Tune** | Auto-tuning active | Special algorithm | Parameter optimization |
| **Auto** | Adaptive mode selection | Dynamic | Future implementation |

## Tuning Methods

| Method | Description | Best For | Parameters |
|--------|-------------|----------|------------|
| **ZieglerNichols** | Classic tuning method | General purpose | Ku, Tu |
| **CohenCoon** | Improved ZN with dead time | Slow processes | Ku, Tu, T, L |
| **IMC** | Internal model control | Fast response | Lambda parameter |
| **TyreusLuyben** | Robust tuning | Stability priority | Ku, Tu |
| **LambdaTuning** | Dead time compensation | Processes with delay | T, L, lambda |

## Oscillation Modes

| Mode | Oscillation Range | Use Case |
|------|-------------------|----------|
| **Normal** | Full range (Min-Max) | Standard auto-tuning |
| **Half** | Half range (±50%) | Reduced disturbance |
| **Mild** | Quarter range (±25%) | Minimal oscillation |

## Enumerations

### TuningMethod
```cpp
enum class TuningMethod {
    ZieglerNichols,  // Classic ZN method
    CohenCoon,      // Improved ZN with dead time
    IMC,            // Internal Model Control
    TyreusLuyben,   // Robust tuning
    LambdaTuning    // Dead time compensation
};
```

### OperationalMode
```cpp
enum class OperationalMode {
    Normal,     // Standard PID
    Reverse,    // Cooling systems
    Manual,     // Direct control
    Override,   // Fixed output
    Track,      // Reference following
    Hold,       // Freeze output
    Preserve,   // Minimal calc
    Tune,       // Auto-tuning
    Auto        // Adaptive (future)
};
```

### OscillationMode
```cpp
enum class OscillationMode {
    Normal,     // Full oscillation
    Half,       // Half oscillation
    Mild        // Mild oscillation
};
```

## Error Codes and Troubleshooting

### Common Issues

| Issue | Symptom | Solution |
|-------|---------|----------|
| **No Output Change** | Output stays constant | Check setpoint range, input scaling |
| **Violent Oscillation** | Output swings wildly | Reduce PID gains, enable filtering |
| **Slow Response** | Output changes slowly | Increase proportional gain, check timing |
| **Integral Windup** | Output stuck at limit | Enable anti-windup protection |
| **Noise Sensitivity** | Erratic output | Enable input filtering, increase derivative |

### Method Return Values

| Method | Return Type | Range | Notes |
|--------|-------------|-------|-------|
| `getOutput()` | float | [minOutput, maxOutput] | Constrained to limits |
| `getKp/Ki/Kd()` | float | [0, ∞) | Current gain values |
| `getKu()` | float | [0, ∞) | Ultimate gain (after tuning) |
| `getTu()` | float | [0, ∞) | Oscillation period (after tuning) |
| `getSetpoint()` | float | (-∞, ∞) | Target value |

### Memory Usage

| Component | RAM Usage | Notes |
|-----------|-----------|-------|
| Base instance | ~400 bytes | Core PID state |
| Auto-tuning | +200 bytes | Ku, Tu, process parameters |
| Filters | +100 bytes | Input/output filtering |
| Anti-windup | +50 bytes | Windup protection |
| **Total** | **~750 bytes** | Typical usage |

## Performance Specifications

- **Update Frequency**: 10Hz (100ms intervals)
- **Calculation Time**: < 1ms per update
- **Memory Footprint**: ~750 bytes RAM
- **Flash Usage**: ~9KB
- **Platform Support**: Arduino AVR, ESP32, and compatible boards

## Thread Safety

**Not thread-safe**: AutoTunePID is designed for single-threaded Arduino environments. All state variables are shared and not protected against concurrent access.

## Best Practices

1. **Initialize properly**: Always set setpoint and gains before use
2. **Constrain inputs**: Ensure process variables are in expected ranges
3. **Monitor output**: Watch for saturation and windup conditions
4. **Tune gradually**: Start with conservative gains and adjust
5. **Use filtering**: Enable input/output filtering for noisy signals
6. **Choose appropriate modes**: Select operational modes based on application needs
