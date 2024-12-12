/*
 * This example demonstrates how to use the AutoTunePID library to maintain a target light intensity.
 *
 * - A light sensor (e.g., photodiode or LDR) is used to measure the ambient light level.
 * - The target intensity (setpoint) is specified in the code.
 * - The AutoTunePID library automatically adjusts the PID values for accurate light control.
 * - The PID output is inverted and used to adjust the brightness of an LED via PWM.
 * - Applications include automatic dimming systems, smart lighting, and energy-efficient designs.
 */


#include <AutoTunePID.h>

// Define pins
#define lightSensorPin A0
#define ledPin 3

// Create PID instance
AutoTunePID pid(255, 0, 10000);

void setup() {
    pinMode(ledPin, OUTPUT);
    Serial.begin(9600);

    // Set PID tunings
    pid.setTunings(1.0, 0.2, 0.1); // Kp, Ki, Kd
}

void loop() {
    // Read light intensity from photodiode (inverted, lower values = more light)
    float currentLight = analogRead(lightSensorPin);

    // Compute PID output to control LED brightness
    float output = pid.compute(currentLight, 512.0); // Target light level: 512 (mid-range)

    // Send the output to the LED (inverted to dim when brighter)
    analogWrite(ledPin, 255 - output);

    // Debugging info
    Serial.print("Light Intensity: ");
    Serial.print(currentLight);
    Serial.print(" | PID Output: ");
    Serial.println(output);

    delay(50); // Sampling delay
}
