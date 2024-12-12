// Example 2: Output Filtering Only
// Useful for stable sensors but sensitive actuators

#include "AutoTunePID.h"

const int SENSOR_PIN = A0;
const int OUTPUT_PIN = 9;
const float SETPOINT = 100.0;
const float OUTPUT_FILTER_ALPHA = 0.15;  // Moderate smoothing for output

AutoTunePID pid(0, 255, ZieglerNichols);

void setup() {
    Serial.begin(115200);
    pid.setSetpoint(SETPOINT);

    // Enable only output filtering
    pid.enableOutputFilter(OUTPUT_FILTER_ALPHA);

    // Set manual PID gains
    pid.setManualGains(2.0, 0.5, 0.1);

    pinMode(OUTPUT_PIN, OUTPUT);
    Serial.println("Time,Input,RawOutput,FilteredOutput,Setpoint");
}

void loop() {
    // Read sensor directly (no filtering)
    float input = analogRead(SENSOR_PIN);

    // Update PID
    pid.update(input);

    // Get filtered output
    float filteredOutput = pid.getOutput();

    // Calculate raw output for comparison
    float rawOutput = pid.getKp() * (SETPOINT - input) +
                     pid.getKi() * pid.getKi() +
                     pid.getKd() * (input - prevInput);

    // Apply filtered output
    analogWrite(OUTPUT_PIN, filteredOutput);

    // Print data for visualization
    Serial.print(millis());
    Serial.print(",");
    Serial.print(input);
    Serial.print(",");
    Serial.print(rawOutput);
    Serial.print(",");
    Serial.print(filteredOutput);
    Serial.print(",");
    Serial.println(SETPOINT);

    delay(50);
}
