#include <AutoTunePID.h>

// Use the atp namespace
using namespace atp;

// Pin definitions
const int inputPin = A0;
const int outputPin = 9;

// PID parameters
float setpoint = 100.0f;
AutoTunePID pid(0.0f, 255.0f, TuningMethod::Manual);

void setup()
{
    Serial.begin(9600);
    pinMode(inputPin, INPUT);
    pinMode(outputPin, OUTPUT);

    pid.setSetpoint(setpoint);
    pid.setManualGains(2.0f, 0.5f, 1.0f); // Set manual gains (Kp, Ki, Kd)
}

void loop()
{
    float currentInput = static_cast<float>(analogRead(inputPin));
    pid.update(currentInput);

    float output = pid.getOutput();
    analogWrite(outputPin, static_cast<int>(output));

    Serial.print("Input: ");
    Serial.print(currentInput);
    Serial.print(" | Output: ");
    Serial.print(output);
    Serial.print(" | Kp: ");
    Serial.print(pid.getKp());
    Serial.print(" | Ki: ");
    Serial.print(pid.getKi());
    Serial.print(" | Kd: ");
    Serial.print(pid.getKd());
    Serial.print(" | Mode: ");

    // Print the current operational mode
    switch (pid.getOperationalMode()) {
    case OperationalMode::Normal:
        Serial.print("Normal");
        break;
    case OperationalMode::Reverse:
        Serial.print("Reverse");
        break;
    case OperationalMode::Manual:
        Serial.print("Manual");
        break;
    case OperationalMode::Override:
        Serial.print("Override");
        break;
    case OperationalMode::Track:
        Serial.print("Track");
        break;
    case OperationalMode::Hold:
        Serial.print("Hold");
        break;
    case OperationalMode::Preserve:
        Serial.print("Preserve");
        break;
    case OperationalMode::Tune:
        Serial.print("Tune");
        break;
    case OperationalMode::Auto:
        Serial.print("Auto");
        break;
    default:
        Serial.print("Other");
        break;
    }
    Serial.println();

    delay(100);
}
