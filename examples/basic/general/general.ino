#include <PIDController.h>

// Define pins
#define sensorPin A0
#define actuatorPin 3

// Create PID instance
PIDController pid(255, 0, 10000); // Max, Min, Auto-tuning duration

void setup() {
    pinMode(actuatorPin, OUTPUT);
    pid.setTunings(2.0, 0.5, 0.1); // Predefined tunings
    pid.setSetpointRamp(0.1); // Smooth ramp
}

void loop() {
    float currentTemperature = analogRead(sensorPin) * (5.0 / 1023.0) * 100;
    float output = pid.compute(currentTemperature, 25.0); // Target temperature 25°C
    analogWrite(actuatorPin, output);
}
