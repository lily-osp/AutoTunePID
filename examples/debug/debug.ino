#include <AutoTunePID.h>

AutoTunePID pid(0, 255); // minOutput = 0, maxOutput = 255

void setup()
{
    Serial.begin(9600);
    pid.setSetpoint(100); // Set the desired setpoint
    pid.setTuningMethod(TuningMethod::ZieglerNichols); // Use Ziegler-Nichols method
}

void loop()
{
    float currentInput = analogRead(A0) * 0.1; // Simulate input (adjust scaling as needed)
    pid.update(currentInput);

    // Debug prints
    Serial.print("Input: ");
    Serial.print(currentInput);
    Serial.print(" | Output: ");
    Serial.print(pid.getOutput());
    Serial.print(" | Kp: ");
    Serial.print(pid.getKp());
    Serial.print(" | Ki: ");
    Serial.print(pid.getKi());
    Serial.print(" | Kd: ");
    Serial.print(pid.getKd());
    Serial.print(" | Ku: ");
    Serial.print(pid.getKu());
    Serial.print(" | Tu: ");
    Serial.print(pid.getTu());
    Serial.print(" | Amplitude: ");
    Serial.print(pid.getAmplitude());
    Serial.print(" | Last Input: ");
    Serial.print(pid.getLastInput());
    Serial.print(" | Peak 0: ");
    Serial.println(pid.getPeak(0));

    delay(100); // Adjust delay as needed
}
