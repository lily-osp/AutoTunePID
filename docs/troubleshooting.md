# Troubleshooting Guide

This guide helps you diagnose and solve common issues with the AutoTunePID library. Follow the systematic approach below to identify and resolve problems.

## Quick Diagnosis Checklist

### Before You Start
- [ ] Arduino IDE version 1.5.x or later
- [ ] AutoTunePID library properly installed
- [ ] Correct board selected in Arduino IDE
- [ ] Serial monitor baud rate matches sketch (usually 9600)
- [ ] Power supply adequate for your actuators

### Basic Functionality Test
Upload this minimal test sketch:
```cpp
#include "AutoTunePID.h"

AutoTunePID pid(0, 255, TuningMethod::Manual);
bool ledState = false;

void setup() {
    pinMode(13, OUTPUT);
    pid.setSetpoint(100);
    pid.setManualGains(1.0, 0.1, 0.01);
    Serial.begin(9600);
}

void loop() {
    // Simple test input
    float testInput = 50 + 25 * sin(millis() / 1000.0);

    pid.update(testInput);
    float output = pid.getOutput();

    Serial.print("Input: ");
    Serial.print(testInput);
    Serial.print(" | Output: ");
    Serial.println(output);

    // Visual feedback
    digitalWrite(13, (output > 127) ? HIGH : LOW);

    delay(100);
}
```

**Expected Result**: Serial monitor shows changing input/output values, LED blinks based on output.

## Common Issues and Solutions

## Issue 1: No Output Response

### Symptoms
- Output stays at 0 or minimum value
- No change when setpoint or input changes
- Actuator doesn't respond

### Possible Causes & Solutions

#### Cause: Incorrect Pin Configuration
```cpp
// PROBLEM: Wrong pin mode
pinMode(sensorPin, OUTPUT); // Should be INPUT!

// SOLUTION: Correct pin modes
pinMode(sensorPin, INPUT);
pinMode(actuatorPin, OUTPUT);
```

#### Cause: Wrong Output Range
```cpp
// PROBLEM: Range doesn't match actuator
AutoTunePID pid(0, 100, TuningMethod::Manual); // For servo
analogWrite(pwmPin, pid.getOutput()); // PWM expects 0-255!

// SOLUTION: Match ranges
AutoTunePID pid(0, 255, TuningMethod::Manual); // For PWM
// or scale output for servos
servo.write(map(pid.getOutput(), 0, 255, 0, 180));
```

#### Cause: Setpoint Out of Range
```cpp
// PROBLEM: Setpoint doesn't match input scale
float temperature = analogRead(A0) * (100.0 / 1023.0); // 0-100 range
pid.setSetpoint(250); // Way outside input range!

// SOLUTION: Match setpoint to input range
pid.setSetpoint(50); // Reasonable target within 0-100
```

## Issue 2: Violent Oscillation

### Symptoms
- Output swings wildly between min and max
- System becomes unstable
- Actuator chatters or moves erratically

### Solutions

#### Solution 1: Reduce PID Gains
```cpp
// PROBLEM: Gains too high
pid.setManualGains(10.0, 2.0, 1.0); // Too aggressive!

// SOLUTION: Conservative starting values
pid.setManualGains(1.0, 0.1, 0.01); // Much gentler
```

#### Solution 2: Enable Filtering
```cpp
// PROBLEM: Noisy input signal
pid.update(analogRead(A0)); // Raw ADC noise!

// SOLUTION: Add input filtering
pid.enableInputFilter(0.1); // Smooth input signal
pid.enableOutputFilter(0.2); // Smooth output changes
```

#### Solution 3: Check Control Loop Timing
```cpp
// PROBLEM: Too fast updates
void loop() {
    pid.update(input);
    delay(10); // 100Hz - too fast!
}

// SOLUTION: Appropriate timing
void loop() {
    pid.update(input);
    delay(100); // 10Hz - good for most systems
}
```

#### Solution 4: Use Appropriate Tuning Method
```cpp
// PROBLEM: Wrong tuning for your system
AutoTunePID pid(0, 255, TuningMethod::ZieglerNichols); // For fast systems

// SOLUTION: Choose based on system characteristics
AutoTunePID pid(0, 255, TuningMethod::CohenCoon); // For slow systems
```

## Issue 3: Slow or No Response

### Symptoms
- Output changes very slowly
- System never reaches setpoint
- Actuator barely moves

### Solutions

#### Solution 1: Increase Proportional Gain
```cpp
// PROBLEM: P gain too low
pid.setManualGains(0.1, 0.5, 0.1); // Weak response

// SOLUTION: Increase P gain gradually
pid.setManualGains(2.0, 0.5, 0.1); // Better response
```

#### Solution 2: Check Input Scaling
```cpp
// PROBLEM: Wrong input scaling
float level = analogRead(A0); // 0-1023
pid.setSetpoint(500); // Half of 1023
// But level is used as-is, setpoint is 500

// SOLUTION: Consistent scaling
float level = analogRead(A0) * (100.0 / 1023.0); // Scale to 0-100
pid.setSetpoint(50); // 50% of 100
```

#### Solution 3: Verify Actuator Connection
```cpp
// PROBLEM: Actuator not responding
analogWrite(pin, pid.getOutput()); // PWM pin correct?

// SOLUTION: Test actuator directly
analogWrite(pin, 128); // Half power test
delay(1000);
analogWrite(pin, 0); // Off
```

## Issue 4: Integral Windup

### Symptoms
- Output stuck at minimum or maximum
- System slow to recover from disturbances
- Integral term accumulates excessively

### Solutions

#### Solution 1: Enable Anti-Windup
```cpp
// PROBLEM: No windup protection
pid.update(input); // Integral keeps growing!

// SOLUTION: Enable anti-windup
pid.enableAntiWindup(true, 0.8); // 80% threshold
```

#### Solution 2: Reset Integral Manually
```cpp
// PROBLEM: Stuck in windup
// SOLUTION: Reset when needed
if (abs(pid.getOutput() - pid.getSetpoint()) < 1.0) {
    // Near setpoint, reset integral
    pid.setOperationalMode(OperationalMode::Hold);
    delay(100);
    pid.setOperationalMode(OperationalMode::Normal);
}
```

## Issue 5: Auto-Tuning Problems

### Symptoms
- Auto-tuning never completes
- Wrong gain values calculated
- System oscillates unpredictably

### Solutions

#### Solution 1: Check Oscillation Mode
```cpp
// PROBLEM: Too much oscillation
pid.setOscillationMode(OscillationMode::Normal); // Full oscillation

// SOLUTION: Reduce disturbance
pid.setOscillationMode(OscillationMode::Half); // Less aggressive
```

#### Solution 2: Proper Initial Conditions
```cpp
// PROBLEM: Starting from wrong point
pid.setSetpoint(currentValue + 10); // Too close to current

// SOLUTION: Sufficient initial error
pid.setSetpoint(currentValue + 50); // Good separation
```

#### Solution 3: Monitor Tuning Progress
```cpp
// Add debugging during tuning
if (pid.getOperationalMode() == OperationalMode::Tune) {
    Serial.print("Tuning... Ku: ");
    Serial.print(pid.getKu());
    Serial.print(" Tu: ");
    Serial.println(pid.getTu());
}
```

## Issue 6: Operational Mode Problems

### Symptoms
- Wrong behavior for heating/cooling systems
- Manual mode not working
- Override mode stuck

### Solutions

#### Solution 1: Correct Mode for Application
```cpp
// PROBLEM: Wrong mode for cooling
pid.setOperationalMode(OperationalMode::Normal); // For heating
// But you have a cooler!

// SOLUTION: Use Reverse for cooling
pid.setOperationalMode(OperationalMode::Reverse); // For cooling systems
```

#### Solution 2: Manual Mode Setup
```cpp
// PROBLEM: Manual mode not working
pid.setOperationalMode(OperationalMode::Manual);
// Forgot to set output!

// SOLUTION: Set manual output
pid.setOperationalMode(OperationalMode::Manual);
pid.setManualOutput(75.0); // 75% output
```

#### Solution 3: Override Mode Usage
```cpp
// PROBLEM: Override not working
pid.setOverrideOutput(255);
// Forgot to switch to override mode!

// SOLUTION: Switch to override mode first
pid.setOperationalMode(OperationalMode::Override);
pid.setOverrideOutput(255);
```

## Advanced Diagnostics

### Serial Debug Output
Add comprehensive debugging to your sketch:
```cpp
void loop() {
    float input = readProcessVariable();
    pid.update(input);

    Serial.print("SP: "); Serial.print(pid.getSetpoint());
    Serial.print(" | PV: "); Serial.print(input);
    Serial.print(" | OUT: "); Serial.print(pid.getOutput());
    Serial.print(" | ERR: "); Serial.print(pid.getSetpoint() - input);
    Serial.print(" | MODE: ");

    switch(pid.getOperationalMode()) {
        case OperationalMode::Normal: Serial.print("Normal"); break;
        case OperationalMode::Reverse: Serial.print("Reverse"); break;
        case OperationalMode::Manual: Serial.print("Manual"); break;
        case OperationalMode::Tune: Serial.print("Tune"); break;
        default: Serial.print("Other");
    }

    Serial.println();
    delay(100);
}
```

### Performance Monitoring
```cpp
unsigned long lastTime = 0;
unsigned long loopCount = 0;

void loop() {
    // Your control code here

    loopCount++;
    if (millis() - lastTime >= 1000) {
        Serial.print("Loops/sec: ");
        Serial.println(loopCount);
        loopCount = 0;
        lastTime = millis();
    }
}
```

## System-Specific Issues

### Temperature Control
- **Overshoot**: Reduce proportional gain, enable derivative
- **Slow response**: Increase integral gain gradually
- **Oscillation**: Add input filtering, check sensor noise

### Motor Speed Control
- **Cogging**: Enable output filtering
- **Load changes**: Increase integral gain
- **Starting issues**: Use Track mode for smooth startup

### Position Control
- **Hunting**: Reduce proportional gain, increase derivative
- **Backlash**: Use minimal integral gain
- **Resolution**: Ensure adequate position feedback

## Getting Help

### Information to Provide
When reporting issues, include:
- Arduino board type and IDE version
- Complete sketch code
- Serial output logs
- Expected vs actual behavior
- Sensor/actuator specifications

### Quick Tests
1. **Blink Test**: Verify basic Arduino functionality
2. **Sensor Test**: Confirm sensor readings are correct
3. **Actuator Test**: Verify actuator responds to direct commands
4. **PID Test**: Use manual mode to test direct output control

### Common Mistakes to Avoid
- Forgetting to call `pid.update()` in loop
- Using wrong pin numbers or modes
- Mismatched ranges between PID and actuators
- Too aggressive PID gains on first try
- Not accounting for system delays or lags

## Emergency Procedures

### Safe Shutdown
```cpp
// Immediate safe stop
pid.setOperationalMode(OperationalMode::Override);
pid.setOverrideOutput(0); // Safe output value
```

### Recovery Sequence
```cpp
// Return to normal operation
pid.setOperationalMode(OperationalMode::Track);
pid.setTrackReference(currentSafeValue);
delay(2000); // Allow settling
pid.setOperationalMode(OperationalMode::Normal);
```

Remember: Start simple, add complexity gradually, and always test safety-critical systems thoroughly! 🛡️
