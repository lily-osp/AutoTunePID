// Example 2: Relay Feedback Method
#include "AutoTunePID.h"

const int HEATER_PIN = 9;
const int TEMP_SENSOR_PIN = A0;
const float SAMPLE_TIME_MS = 200;

float currentTemp = 0;
AutoTunePID pid(0, 255, TuningMethod::RelayFeedback);

void setup()
{
    Serial.begin(115200);
    pinMode(HEATER_PIN, OUTPUT);

    pid.setSetpoint(50.0); // Target temperature in Celsius
    pid.enableInputFilter(0.2); // More filtering for temperature
    pid.setTuningDuration(60000); // 60 seconds for temperature system
}

void loop()
{
    currentTemp = readTemperature();

    pid.update(currentTemp);
    analogWrite(HEATER_PIN, pid.getOutput());

    printData();
    delay(SAMPLE_TIME_MS);
}

float readTemperature()
{
    int rawValue = analogRead(TEMP_SENSOR_PIN);
    return (rawValue * 5.0 / 1024.0) * 100; // Convert to Celsius
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
