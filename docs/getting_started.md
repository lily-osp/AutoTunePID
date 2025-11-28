# Getting Started with AutoTunePID

Welcome to the AutoTunePID library! This guide will help you get started quickly with basic PID control. Whether you're new to PID controllers or experienced, this guide will walk you through your first implementation.

## Quick Start (5 Minutes)

### Step 1: Installation

#### Arduino IDE Library Manager (Recommended)
1. Open Arduino IDE
2. Go to **Sketch > Include Library > Manage Libraries**
3. Search for "AutoTunePID"
4. Click **Install**

#### Manual Installation
1. Download the latest release ZIP from [GitHub Releases](https://github.com/lily-osp/AutoTunePID/releases)
2. Open Arduino IDE
3. Go to **Sketch > Include Library > Add .ZIP Library**
4. Select the downloaded ZIP file

### Step 2: Basic PID Control

Here's the simplest way to use AutoTunePID:

```cpp
#include "AutoTunePID.h"

// Create PID controller (min output, max output, tuning method)
AutoTunePID pid(0, 255, TuningMethod::ZieglerNichols);

void setup() {
    // Set your target value (setpoint)
    pid.setSetpoint(100.0); // Target temperature, speed, etc.

    // Optional: Set manual PID gains (skip if using auto-tuning)
    pid.setManualGains(2.0, 0.5, 0.1); // Kp, Ki, Kd
}

void loop() {
    // Read your process variable (temperature, speed, position, etc.)
    float currentValue = analogRead(A0) * (100.0 / 1023.0); // Scale to your units

    // Update PID controller
    pid.update(currentValue);

    // Get control output and apply to your actuator
    float output = pid.getOutput();
    analogWrite(9, output); // PWM output to motor, heater, etc.

    delay(100); // Control loop timing
}
```

### Step 3: Test Your Setup

1. Upload the sketch to your Arduino
2. Monitor the serial output (add `Serial.println(output);` to see values)
3. Adjust the setpoint and observe how the output responds
4. Your basic PID controller is working!

## Understanding the Basics

### What is PID Control?

PID stands for **Proportional-Integral-Derivative**. It's a control algorithm that:

- **Proportional (P)**: Responds to current error
- **Integral (I)**: Corrects accumulated error over time
- **Derivative (D)**: Predicts future error based on rate of change

### Key Concepts

| Term | Description | Your Code |
|------|-------------|-----------|
| **Setpoint (SP)** | Target value you want | `pid.setSetpoint(100.0)` |
| **Process Variable (PV)** | Current measured value | `float currentValue = analogRead(A0)` |
| **Error** | Difference: SP - PV | Calculated automatically |
| **Output** | Control signal to actuator | `pid.getOutput()` |
| **PID Gains** | Control sensitivity | `pid.setManualGains(Kp, Ki, Kd)` |

### Tuning Methods

Choose based on your application:

| Method | Best For | Description |
|--------|----------|-------------|
| **Ziegler-Nichols** | General purpose | Good starting point for most systems |
| **Cohen-Coon** | Processes with dead time | Better for slow-responding systems |
| **IMC** | Fast response needed | Balances speed and stability |
| **Manual Tuning** | Fine control | When you know your system well |

## Common Applications

### Temperature Control

```cpp
// Heater control (Normal mode)
AutoTunePID heaterPID(0, 255, TuningMethod::ZieglerNichols);
heaterPID.setSetpoint(25.0); // Target temperature in °C

// In loop:
float currentTemp = readTemperatureSensor();
heaterPID.update(currentTemp);
analogWrite(heaterPin, heaterPID.getOutput());
```

### Motor Speed Control

```cpp
// DC motor control
AutoTunePID motorPID(0, 255, TuningMethod::CohenCoon);
motorPID.setSetpoint(1500); // Target RPM

// In loop:
float currentRPM = measureMotorRPM();
motorPID.update(currentRPM);
analogWrite(motorPin, motorPID.getOutput());
```

### Position Control

```cpp
// Servo position control
AutoTunePID servoPID(0, 180, TuningMethod::Manual);
servoPID.setManualGains(1.2, 0.1, 0.05);
servoPID.setSetpoint(90); // Target angle

// In loop:
float currentAngle = readServoPosition();
servoPID.update(currentAngle);
servo.write(pid.getOutput());
```

## Next Steps

### For Beginners
1. **Try the Examples**: Look at `examples/BasicPID/BasicPID.ino`
2. **Learn Manual Tuning**: Read `docs/manual.md`
3. **Understand Auto-Tuning**: See `examples/ZieglerNichols/`

### For Advanced Users
1. **Operational Modes**: Check `docs/usage.md` for all control modes
2. **Signal Filtering**: Learn about anti-windup and filtering
3. **System Architecture**: Read `docs/system_architecture.md`

### Troubleshooting
- **Output not responding**: Check your actuator wiring and power
- **Oscillating wildly**: Reduce PID gains or use different tuning method
- **No change in output**: Verify your setpoint and input ranges

## Need Help?

- **Examples**: Check `examples/` folder for complete working sketches
- **Documentation**: See all docs in `docs/` folder
- **Issues**: Report bugs on [GitHub Issues](https://github.com/lily-osp/AutoTunePID/issues)

**Happy controlling!** 🎛️🤖
