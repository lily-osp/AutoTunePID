/*
 * Auto-Tune Liquid Flow Rate Control
 *
 * - A flow meter measures the liquid flow rate (liters per minute).
 * - AutoTunePID dynamically tunes PID parameters.
 * - Smoothing is applied to the flow sensor input for stability.
 * - The PID output adjusts a valve using PWM.
 * - Applications include irrigation systems or industrial fluid control.
 */

#include <AutoTunePID.h>

// Define pins
#define flowSensorPin A0
#define valvePin 3

// Create PID instance
AutoTunePID pid(255, 0, 15000); // PWM range for valve control

void setup() {
    pinMode(valvePin, OUTPUT);
    Serial.begin(9600);

    // Enable auto-tuning
    pid.enableAutoTune(true);
    pid.setFilter(0.15); // Low-pass filter to smooth input noise
}

float getFlowRate() {
    // Simulate flow rate reading (replace with actual sensor code)
    return analogRead(flowSensorPin) * (10.0 / 1023.0); // Convert to L/min
}

void loop() {
    // Read and filter the current flow rate
    float rawFlowRate = getFlowRate();
    float currentFlowRate = pid.filterInput(rawFlowRate);

    // Compute PID output to control valve
    float targetFlowRate = 5.0; // Desired flow rate in L/min
    float output = pid.compute(currentFlowRate, targetFlowRate);

    // Adjust valve position
    analogWrite(valvePin, constrain(output, 0, 255));

    // Debugging info
    Serial.print("Current Flow Rate: ");
    Serial.print(currentFlowRate);
    Serial.print(" L/min | Target Flow Rate: ");
    Serial.print(targetFlowRate);
    Serial.print(" L/min | PID Output: ");
    Serial.println(output);

    delay(200); // Sampling delay
}
