#include <AutoTunePID.h>

// Pin definitions
const int inputPin = A0;
const int outputPin = 9;

// PID parameters
float setpoint = 100.0;
AutoTunePID pid(0, 255, TuningMethod::ZieglerNichols);

void setup()
{
    Serial.begin(9600);
    pinMode(inputPin, INPUT);
    pinMode(outputPin, OUTPUT);

    pid.setSetpoint(setpoint);
    pid.enableAntiWindup(true, 0.8); // Enable anti-windup with threshold = 0.8
}

void loop()
{
    float currentInput = analogRead(inputPin);
    pid.update(currentInput);

    float output = pid.getOutput();
    analogWrite(outputPin, output);

    Serial.print("Input: ");
    Serial.print(currentInput);
    Serial.print(" | Output: ");
    Serial.print(output);
    Serial.print(" | Kp: ");
    Serial.print(pid.getKp());
    Serial.print(" | Ki: ");
    Serial.print(pid.getKi());
    Serial.print(" | Kd: ");
    Serial.println(pid.getKd());

    delay(100);
}