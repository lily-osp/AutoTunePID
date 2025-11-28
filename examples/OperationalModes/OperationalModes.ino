#include "AutoTunePID.h"

// Pin definitions
const int sensorPin = A0;    // Analog input for process variable
const int outputPin = 9;     // PWM output for control signal

// PID controller instance
AutoTunePID pid(0, 255, TuningMethod::ZieglerNichols);

void setup() {
    Serial.begin(9600);
    pinMode(sensorPin, INPUT);
    pinMode(outputPin, OUTPUT);

    // Configure PID for normal operation
    pid.setSetpoint(50.0); // Target value of 50 (analog units)
    pid.setManualGains(2.0, 0.5, 0.1); // Sample PID gains

    Serial.println("Operational Modes Demonstration");
    Serial.println("Commands:");
    Serial.println("n - Normal PID mode");
    Serial.println("r - Reverse PID mode");
    Serial.println("m - Manual mode (set output to 50%)");
    Serial.println("o - Override mode (set output to 100)");
    Serial.println("t - Track mode (follow reference ramp)");
    Serial.println("h - Hold mode (freeze current output)");
    Serial.println("p - Preserve mode (minimal calculations)");
}

void loop() {
    // Read process variable (0-1023 range)
    float processVariable = analogRead(sensorPin) * (100.0 / 1023.0);

    // Update PID controller
    pid.update(processVariable);

    // Get output and apply to control pin
    float output = pid.getOutput();
    analogWrite(outputPin, output);

    // Display current status
    Serial.print("PV: ");
    Serial.print(processVariable, 1);
    Serial.print(" | SP: ");
    Serial.print(pid.getSetpoint(), 1);
    Serial.print(" | OUT: ");
    Serial.print(output, 1);

    // Display current operational mode
    Serial.print(" | Mode: ");
    switch(pid.getOperationalMode()) {
        case OperationalMode::Normal: Serial.print("Normal"); break;
        case OperationalMode::Reverse: Serial.print("Reverse"); break;
        case OperationalMode::Manual: Serial.print("Manual"); break;
        case OperationalMode::Override: Serial.print("Override"); break;
        case OperationalMode::Track: Serial.print("Track"); break;
        case OperationalMode::Hold: Serial.print("Hold"); break;
        case OperationalMode::Preserve: Serial.print("Preserve"); break;
        case OperationalMode::Tune: Serial.print("Tune"); break;
        case OperationalMode::Auto: Serial.print("Auto"); break;
    }

    Serial.println();

    // Check for serial commands to change modes
    if (Serial.available()) {
        char command = Serial.read();

        switch(command) {
            case 'n':
                pid.setOperationalMode(OperationalMode::Normal);
                Serial.println("Switched to Normal PID mode");
                break;

            case 'r':
                pid.setOperationalMode(OperationalMode::Reverse);
                Serial.println("Switched to Reverse PID mode");
                break;

            case 'm':
                pid.setOperationalMode(OperationalMode::Manual);
                pid.setManualOutput(50.0); // 50% output
                Serial.println("Switched to Manual mode (50% output)");
                break;

            case 'o':
                pid.setOperationalMode(OperationalMode::Override);
                pid.setOverrideOutput(255.0); // Maximum output
                Serial.println("Switched to Override mode (max output)");
                break;

            case 't':
                pid.setOperationalMode(OperationalMode::Track);
                Serial.println("Switched to Track mode - ramping reference");
                // Start track mode demonstration
                for(float ref = 0; ref <= 255; ref += 25.5) {
                    pid.setTrackReference(ref);
                    delay(500);
                }
                break;

            case 'h':
                pid.setOperationalMode(OperationalMode::Hold);
                Serial.println("Switched to Hold mode (output frozen)");
                break;

            case 'p':
                pid.setOperationalMode(OperationalMode::Preserve);
                Serial.println("Switched to Preserve mode (minimal calc)");
                break;
        }
    }

    delay(1000); // Update every second
}

/*
 * Operational Modes Demonstration
 *
 * This example shows how to use all available operational modes:
 *
 * 1. Normal: Standard PID control (error = setpoint - input)
 * 2. Reverse: Reverse PID control for cooling systems (error = input - setpoint)
 * 3. Manual: Direct output control (0-100% range)
 * 4. Override: Emergency override with fixed output value
 * 5. Track: Output follows a reference signal
 * 6. Hold: Freezes current output value
 * 7. Preserve: Minimal calculations with error monitoring
 * 8. Tune: Auto-tuning mode (not demonstrated here)
 * 9. Auto: Automatic mode selection (reserved for future)
 *
 * Use serial monitor to send commands and see mode changes in action.
 */
