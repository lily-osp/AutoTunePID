// Example 3: Manual Tuning with Switchable Modes
// This example allows switching between auto and manual tuning via Serial commands

#include "AutoTunePID.h"

const int PROCESS_INPUT_PIN = A0;
const int PROCESS_OUTPUT_PIN = 11;

AutoTunePID pid(0, 255, ZieglerNichols);
String command;

void setup() {
  Serial.begin(115200);
  pinMode(PROCESS_OUTPUT_PIN, OUTPUT);

  pid.setSetpoint(500); // Example setpoint
  Serial.println("Commands: 'auto' for auto-tuning, 'manual Kp Ki Kd' for manual tuning");
}

void loop() {
  // Check for serial commands
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    processCommand(command);
  }

  float processValue = analogRead(PROCESS_INPUT_PIN);
  pid.update(processValue);
  float output = pid.getOutput();
  analogWrite(PROCESS_OUTPUT_PIN, output);

  // Print status every second
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    printStatus(processValue, output);
    lastPrint = millis();
  }
}

void processCommand(String cmd) {
  if (cmd.startsWith("auto")) {
    pid.setTuningMethod(ZieglerNichols);
    Serial.println("Switching to auto-tuning");
  }
  else if (cmd.startsWith("manual")) {
    // Parse manual PID values: "manual 2.0 0.5 0.1"
    float kp = cmd.substring(7).toFloat();
    int ki_start = cmd.indexOf(' ', 7) + 1;
    float ki = cmd.substring(ki_start).toFloat();
    int kd_start = cmd.indexOf(' ', ki_start) + 1;
    float kd = cmd.substring(kd_start).toFloat();

    pid.setManualGains(kp, ki, kd);
    Serial.println("Switching to manual tuning");
  }
}

void printStatus(float input, float output) {
  Serial.print("Input: "); Serial.print(input);
  Serial.print(" Output: "); Serial.print(output);
  Serial.print(" Kp: "); Serial.print(pid.getKp());
  Serial.print(" Ki: "); Serial.print(pid.getKi());
  Serial.print(" Kd: "); Serial.println(pid.getKd());
}
