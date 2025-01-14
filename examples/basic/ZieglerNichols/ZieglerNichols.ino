// Example 1: Ziegler-Nichols Method
#include "AutoTunePID.h"

const int ENCODER_PIN = 2;
const int MOTOR_PIN = 10;
const float PULSES_PER_REV = 24.0;
const float SAMPLE_TIME_MS = 100;

volatile long encoderCount = 0;
float currentSpeed = 0;

AutoTunePID pid(0, 255, TuningMethod::ZieglerNichols);

void setup()
{
    Serial.begin(115200);
    pinMode(MOTOR_PIN, OUTPUT);
    pinMode(ENCODER_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countPulse, RISING);

    pid.setSetpoint(100.0);
    pid.enableOutputFilter(0.15);
    pid.setTuningDuration(30000); // 30 seconds for tuning
}

void loop()
{
    currentSpeed = (encoderCount * (60000.0 / SAMPLE_TIME_MS)) / PULSES_PER_REV;
    encoderCount = 0;

    pid.update(currentSpeed);
    analogWrite(MOTOR_PIN, pid.getOutput());

    printData();
    delay(SAMPLE_TIME_MS);
}

void countPulse()
{
    encoderCount++;
}

// printing function
void printData()
{
    if (pid.isTuning()) {
        Serial.print("Tuning - ");
    }
    Serial.print("PV:");
    Serial.print(currentSpeed);
    Serial.print(" SP:");
    Serial.print(pid.getSetpoint());
    Serial.print(" OUT:");
    Serial.println(pid.getOutput());

    static bool gainsReported = false;
    if (!pid.isTuning() && !gainsReported) {
        Serial.println("\nTuning Complete!");
        Serial.print("Kp:");
        Serial.println(pid.getKp(), 4);
        Serial.print("Ki:");
        Serial.println(pid.getKi(), 4);
        Serial.print("Kd:");
        Serial.println(pid.getKd(), 4);
        gainsReported = true;
    }
}
