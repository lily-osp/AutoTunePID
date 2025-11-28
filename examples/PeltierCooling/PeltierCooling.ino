#include "AutoTunePID.h"

// Pin definitions
const int tempSensorPin = A0; // Analog input pin for temperature sensor
const int peltierPin = 9;     // PWM output pin for Peltier cell control

// PID parameters
float setpoint = 12.0; // Desired temperature in °C
AutoTunePID pid(0, 255, TuningMethod::ZieglerNichols); // Create PID controller

void setup()
{
    Serial.begin(9600); // Initialize serial communication
    pinMode(tempSensorPin, INPUT); // Set temperature sensor pin as input
    pinMode(peltierPin, OUTPUT);   // Set Peltier control pin as output

    // Configure PID controller for cooling system
    pid.setSetpoint(setpoint); // Set target temperature to 12°C
    pid.enableInputFilter(0.1);  // Enable input filtering for stable readings
    pid.enableAntiWindup(true, 0.8); // Enable anti-windup protection
    pid.setOperationalMode(OperationalMode::Reverse); // Use Reverse mode for cooling

    Serial.println("Peltier Cooling System Initialized");
    Serial.print("Target Temperature: ");
    Serial.print(setpoint);
    Serial.println("°C");
}

void loop()
{
    // Read temperature sensor (assuming 0-100°C range, adjust formula as needed)
    float currentTemp = analogRead(tempSensorPin) * (100.0 / 1023.0);

    // Update PID controller
    pid.update(currentTemp);

    // Get the computed output (0-255 for PWM control)
    float output = pid.getOutput();

    // Apply output to Peltier cell (PWM control)
    analogWrite(peltierPin, output);

    // Print debugging information
    Serial.print("Temperature: ");
    Serial.print(currentTemp);
    Serial.print("°C | Setpoint: ");
    Serial.print(setpoint);
    Serial.print("°C | Output: ");
    Serial.print(output);
    Serial.print(" | Error: ");
    Serial.print(pid.getSetpoint() - currentTemp); // Show normal error for reference
    Serial.print(" | Mode: Reverse");

    // Determine cooling status
    if (output > 10) { // Small threshold to avoid noise
        Serial.print(" | Status: COOLING");
    } else {
        Serial.print(" | Status: IDLE");
    }

    Serial.println();

    // Small delay to maintain consistent sample time
    delay(100);
}

/*
 * Peltier Cooling System Explanation:
 *
 * In Reverse mode:
 * - When temperature > setpoint (12°C): Error is positive → Cooling activated
 * - When temperature < setpoint (12°C): Error is negative → Cooling deactivated
 * - When temperature = setpoint (12°C): Error is zero → Cooling deactivated
 *
 * This is perfect for cooling systems where you want to activate cooling
 * when the temperature exceeds the target, and deactivate when it drops below.
 *
 * Example behavior:
 * Temp = 15°C (> 12°C): Error = 15-12 = +3 → Cooling ON
 * Temp = 10°C (< 12°C): Error = 10-12 = -2 → Cooling OFF
 * Temp = 12°C (= 12°C): Error = 0 → Cooling OFF
 */
