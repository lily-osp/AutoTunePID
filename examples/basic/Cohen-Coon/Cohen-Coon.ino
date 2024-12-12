// Example 2: Cohen-Coon Tuning with Output Filtering
// This example uses Cohen-Coon tuning method for a motor speed controller

#include "AutoTunePID.h"

const int ENCODER_PIN = 2;
const int MOTOR_PIN = 10;

volatile long encoderCount = 0;
float currentSpeed = 0;

AutoTunePID pid(0, 255, CohenCoon);

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countPulse, RISING);

  pid.setSetpoint(100.0); // Target speed in RPM
  pid.enableOutputFilter(0.2); // Smooth control output
}

void loop() {
  // Calculate current speed from encoder count
  currentSpeed = (encoderCount * 60.0) / 24; // Convert to RPM
  encoderCount = 0; // Reset counter

  pid.update(currentSpeed);
  float output = pid.getOutput();
  analogWrite(MOTOR_PIN, output);

  Serial.print("Speed: "); Serial.print(currentSpeed);
  Serial.print(" Output: "); Serial.println(output);

  delay(100);
}

void countPulse() {
  encoderCount++;
}
