// Example 3: Combined Input and Output Filtering
// - Input filtering for noisy sensors
// - Output filtering for smooth control
// - Configurable filter parameters for both
// - CSV data output for visualization
// - Interactive filter adjustment via Serial

#include "AutoTunePID.h"

// Pin definitions
const int SENSOR_PIN = A0;    // Analog input from sensor
const int OUTPUT_PIN = 9;      // PWM output pin

// System parameters
const float SETPOINT = 100.0;  // Target value
const float MIN_OUTPUT = 0.0;  // Minimum output value
const float MAX_OUTPUT = 255.0; // Maximum output value

// Filter parameters
const float INPUT_FILTER_ALPHA = 0.1;   // Smaller value = more smoothing
const float OUTPUT_FILTER_ALPHA = 0.15;  // Smaller value = more smoothing

// Create PID controller
AutoTunePID pid(MIN_OUTPUT, MAX_OUTPUT, ZieglerNichols);

// Variables for storing raw values for comparison
float rawInput = 0.0;
float rawOutput = 0.0;

void setup() {
    Serial.begin(115200);

    // Configure PID controller
    pid.setSetpoint(SETPOINT);

    // Enable both input and output filtering
    pid.enableInputFilter(INPUT_FILTER_ALPHA);
    pid.enableOutputFilter(OUTPUT_FILTER_ALPHA);

    // Initial manual PID gains (you can adjust these)
    pid.setManualGains(2.0, 0.5, 0.1);

    // Setup output pin
    pinMode(OUTPUT_PIN, OUTPUT);

    // Print CSV header
    Serial.println("Time,RawInput,FilteredInput,RawOutput,FilteredOutput,Setpoint");
}

void loop() {
    // Read sensor with artificial noise for demonstration
    rawInput = analogRead(SENSOR_PIN);
    // Add synthetic noise for demonstration (remove this in real applications)
    rawInput += random(-20, 20);

    // Update PID controller (input will be filtered internally)
    pid.update(rawInput);

    // Get the controller output (will be filtered internally)
    float filteredOutput = pid.getOutput();

    // Apply the output
    analogWrite(OUTPUT_PIN, filteredOutput);

    // Print data in CSV format for plotting
    Serial.print(millis());
    Serial.print(",");
    Serial.print(rawInput);
    Serial.print(",");
    Serial.print(rawInput * (1 - INPUT_FILTER_ALPHA) + filteredOutput * INPUT_FILTER_ALPHA); // Approximate filtered input
    Serial.print(",");
    Serial.print(rawOutput);
    Serial.print(",");
    Serial.print(filteredOutput);
    Serial.print(",");
    Serial.println(SETPOINT);

    // Store current output as previous for next iteration
    rawOutput = filteredOutput;

    delay(50); // 50ms sampling rate
}

// Optional: Function to demonstrate the effect of different filter values
void adjustFilters() {
    static unsigned long lastCheck = 0;
    const unsigned long CHECK_INTERVAL = 5000; // 5 seconds

    if (millis() - lastCheck >= CHECK_INTERVAL) {
        // Read serial for new filter values
        if (Serial.available() > 0) {
            String command = Serial.readStringUntil('\n');

            // Format: "filter input output"
            // Example: "filter 0.1 0.15"
            if (command.startsWith("filter")) {
                int firstSpace = command.indexOf(' ');
                int secondSpace = command.indexOf(' ', firstSpace + 1);

                if (firstSpace > 0 && secondSpace > 0) {
                    float newInputAlpha = command.substring(firstSpace + 1, secondSpace).toFloat();
                    float newOutputAlpha = command.substring(secondSpace + 1).toFloat();

                    // Update filter values
                    pid.enableInputFilter(newInputAlpha);
                    pid.enableOutputFilter(newOutputAlpha);

                    Serial.print("New filter values - Input: ");
                    Serial.print(newInputAlpha);
                    Serial.print(" Output: ");
                    Serial.println(newOutputAlpha);
                }
            }
        }
        lastCheck = millis();
    }
}
