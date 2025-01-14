// Example 5: Manual Tuning with Real-time Adjustment
#include "AutoTunePID.h"

const int PROCESS_INPUT_PIN = A0;
const int PROCESS_OUTPUT_PIN = 9;
const int KP_POT_PIN = A1;
const int KI_POT_PIN = A2;
const int KD_POT_PIN = A3;
const float SAMPLE_TIME_MS = 100;

float processValue = 0;
AutoTunePID pid(0, 255, TuningMethod::Manual);

void setup()
{
    Serial.begin(115200);
    pinMode(PROCESS_OUTPUT_PIN, OUTPUT);

    pid.setSetpoint(50.0);
    pid.enableInputFilter(0.1);
    pid.enableOutputFilter(0.1);
}

void loop()
{
    // Read PID gains from potentiometers
    float kp = map(analogRead(KP_POT_PIN), 0, 1023, 0, 100) / 10.0; // 0-10.0
    float ki = map(analogRead(KI_POT_PIN), 0, 1023, 0, 1000) / 1000.0; // 0-1.0
    float kd = map(analogRead(KD_POT_PIN), 0, 1023, 0, 2000) / 100.0; // 0-20.0

    pid.setManualGains(kp, ki, kd);

    processValue = analogRead(PROCESS_INPUT_PIN);
    pid.update(processValue);
    analogWrite(PROCESS_OUTPUT_PIN, pid.getOutput());

    // Print data with current gains
    Serial.print("PV:");
    Serial.print(processValue);
    Serial.print(" SP:");
    Serial.print(pid.getSetpoint());
    Serial.print(" OUT:");
    Serial.print(pid.getOutput());
    Serial.print(" Kp:");
    Serial.print(kp);
    Serial.print(" Ki:");
    Serial.print(ki);
    Serial.print(" Kd:");
    Serial.println(kd);

    delay(SAMPLE_TIME_MS);
}
