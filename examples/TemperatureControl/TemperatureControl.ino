/*
 * This example demonstrates how to use the AutoTunePID library to maintain a target temperature.
 *
 * - A temperature sensor (e.g., LM35) is used to measure the current temperature.
 * - The target temperature (setpoint) is specified in the code.
 * - The AutoTunePID library automatically adjusts the PID parameters to ensure accurate control.
 * - The PID output controls an actuator (e.g., heater or fan) using PWM.
 * - Practical applications include climate control, greenhouses, and smart home temperature management.
 */


#include <AutoTunePID.h>

// Define pins
#define sensorPin A0
#define actuatorPin 3

// Create PID instance (max output, min output, auto-tuning duration in ms)
AutoTunePID pid(255, 0, 10000);

void setup() {
    pinMode(actuatorPin, OUTPUT);
    Serial.begin(9600);

    // Set initial tunings (optional if auto-tuning is used)
    pid.setTunings(2.0, 0.5, 0.1); // Kp, Ki, Kd
    pid.setSetpointRamp(0.1);      // Gradually ramp to target setpoint
}

void loop() {
    // Read temperature sensor (LM35 outputs 10mV/°C)
    float currentTemperature = analogRead(sensorPin) * (5.0 / 1023.0) * 100;

    // Compute PID output to control temperature
    float output = pid.compute(currentTemperature, 25.0); // Target temp: 25°C

    // Send the output to the actuator (e.g., heater)
    analogWrite(actuatorPin, output);

    // Debugging info
    Serial.print("Temp: ");
    Serial.print(currentTemperature);
    Serial.print("°C | PID Output: ");
    Serial.println(output);

    delay(100); // Sampling delay
}
