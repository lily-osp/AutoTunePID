// Example 1: Input Filtering Only
// Useful for systems with noisy sensors but stable actuators
#include "AutoTunePID.h"

const int SENSOR_PIN = A0;
const int OUTPUT_PIN = 9;
const float SETPOINT = 100.0;
const float INPUT_FILTER_ALPHA = 0.1;  // Heavy smoothing for noisy input

AutoTunePID pid(0, 255, ZieglerNichols);

void setup() {
    Serial.begin(115200);
    pid.setSetpoint(SETPOINT);

    // Enable only input filtering
    pid.enableInputFilter(INPUT_FILTER_ALPHA);

    // Set manual PID gains
    pid.setManualGains(2.0, 0.5, 0.1);

    pinMode(OUTPUT_PIN, OUTPUT);
    Serial.println("Time,RawInput,FilteredInput,Output,Setpoint");
}

void loop() {
    // Read noisy sensor
    float rawInput = analogRead(SENSOR_PIN);
    // Simulate sensor noise (remove in real applications)
    rawInput += random(-20, 20);

    // Update PID with filtered input
    pid.update(rawInput);
    float output = pid.getOutput();

    // Apply output directly (no filtering)
    analogWrite(OUTPUT_PIN, output);

    // Print data for visualization
    Serial.print(millis());
    Serial.print(",");
    Serial.print(rawInput);
    Serial.print(",");
    Serial.print(rawInput * (1 - INPUT_FILTER_ALPHA) + output * INPUT_FILTER_ALPHA);
    Serial.print(",");
    Serial.print(output);
    Serial.print(",");
    Serial.println(SETPOINT);

    delay(50);
}
