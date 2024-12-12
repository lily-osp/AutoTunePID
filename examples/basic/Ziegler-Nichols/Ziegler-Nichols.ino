// Example 1: Basic Auto-Tuning using Ziegler-Nichols
// This example demonstrates automatic PID tuning for a temperature control system

#include "AutoTunePID.h"

// Define pins
const int SENSOR_PIN = A0;
const int HEATER_PIN = 9;

// Create PID controller instance
// Arguments: min output, max output, tuning method
AutoTunePID pid(0, 255, ZieglerNichols);

void setup() {
  Serial.begin(115200);
  pinMode(HEATER_PIN, OUTPUT);

  // Set target temperature (setpoint)
  pid.setSetpoint(50.0); // 50 degrees

  // Enable input filtering to reduce noise
  pid.enableInputFilter(0.1); // Alpha = 0.1 for moderate smoothing
}

void loop() {
  // Read temperature sensor (example assumes linear conversion)
  float temperature = analogRead(SENSOR_PIN) * 0.1; // Convert to degrees

  // Update PID controller
  pid.update(temperature);

  // Get and apply control output
  float output = pid.getOutput();
  analogWrite(HEATER_PIN, output);

  // Print debug information
  Serial.print("Temperature: "); Serial.print(temperature);
  Serial.print(" Output: "); Serial.print(output);
  Serial.print(" Kp: "); Serial.print(pid.getKp());
  Serial.print(" Ki: "); Serial.print(pid.getKi());
  Serial.print(" Kd: "); Serial.println(pid.getKd());

  delay(100);
}
