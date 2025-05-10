#include <AutoTunePID.h>

// Create a PID controller with output range 0-255 and Ziegler-Nichols tuning
AutoTunePID pid(0, 255, TuningMethod::ZieglerNichols);

void setup() {
  Serial.begin(9600);
  pid.setSetpoint(128); // Target value (midpoint for 8-bit ADC)
  pid.setOperationalMode(OperationalMode::Tune); // Start with auto-tuning
}

void loop() {
  float input = analogRead(A0); // Simulated process variable (0-1023)
  pid.update(input);
  float output = pid.getOutput();
  analogWrite(3, output); // Simulated actuator output

  // Print values to Serial Monitor
  Serial.print("Setpoint: "); Serial.print(pid.getSetpoint());
  Serial.print(" | Input: "); Serial.print(input);
  Serial.print(" | Output: "); Serial.print(output);
  Serial.print(" | Kp: "); Serial.print(pid.getKp());
  Serial.print(" | Ki: "); Serial.print(pid.getKi());
  Serial.print(" | Kd: "); Serial.println(pid.getKd());
  delay(200);
} 