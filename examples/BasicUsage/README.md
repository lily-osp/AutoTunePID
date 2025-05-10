# BasicUsage Example

This example demonstrates the simplest way to use the AutoTunePID library for PID control on Arduino.

## What It Does
- Initializes a PID controller with auto-tuning (Ziegler-Nichols method).
- Reads a simulated process variable from analog pin A0.
- Outputs the PID result to PWM pin 3 (e.g., to control a motor, heater, or LED).
- Prints setpoint, input, output, and PID gains to the Serial Monitor.

## Hardware Setup
- Connect a sensor (e.g., potentiometer or temperature sensor) to analog pin A0.
- Connect an actuator (e.g., LED, motor driver, or heater) to PWM pin 3.
- Make sure to use appropriate resistors or driver circuits for your hardware.

## How to Use
1. Open this example in the Arduino IDE.
2. Upload it to your Arduino board.
3. Open the Serial Monitor at 9600 baud to observe the PID values.
4. Adjust your sensor to see how the PID output responds.

## What to Expect
- The controller will start in auto-tuning mode, then switch to normal PID control.
- The Serial Monitor will show:
  - Setpoint
  - Input (sensor value)
  - Output (PWM value)
  - Kp, Ki, Kd (PID gains)
- You can change the setpoint or tuning method in the code to experiment.

---
For more advanced usage, see the main library [Readme.md](../../Readme.md) and the `docs/` folder. 