// Example: Cohen-Coon Method for Process Control
#include "AutoTunePID.h"

const int ENCODER_PIN = 2;
const int MOTOR_PIN = 10;
const float PULSES_PER_REV = 24.0;
const float SAMPLE_TIME_MS = 100;

volatile long encoderCount = 0;
float currentSpeed = 0;

// Initialize PID with Cohen-Coon tuning method
AutoTunePID pid(0, 255, TuningMethod::CohenCoon);

void setup()
{
    Serial.begin(115200);
    pinMode(MOTOR_PIN, OUTPUT);
    pinMode(ENCODER_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countPulse, RISING);

    // Cohen-Coon typically needs longer tuning duration for better process response
    pid.setTuningDuration(45000); // 45 seconds for tuning
    pid.setSetpoint(100.0); // Target speed in RPM

    // Cohen-Coon works better with some filtering
    pid.enableInputFilter(0.15); // Moderate input filtering
    pid.enableOutputFilter(0.2); // Slightly stronger output filtering
}

void loop()
{
    // Calculate current speed in RPM
    currentSpeed = (encoderCount * (60000.0 / SAMPLE_TIME_MS)) / PULSES_PER_REV;
    encoderCount = 0;

    // Update PID controller
    pid.update(currentSpeed);

    // Apply control output
    analogWrite(MOTOR_PIN, pid.getOutput());

    // Print data using shared printing function
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
