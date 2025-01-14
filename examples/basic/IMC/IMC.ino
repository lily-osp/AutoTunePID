// Example 3: IMC (Internal Model Control) Method
#include "AutoTunePID.h"

const int SERVO_PIN = 5;
const int POSITION_SENSOR_PIN = A0;
const float SAMPLE_TIME_MS = 50;

float currentPosition = 0;
AutoTunePID pid(0, 180, TuningMethod::IMC); // Servo range 0-180

void setup()
{
    Serial.begin(115200);
    pinMode(SERVO_PIN, OUTPUT);

    pid.setSetpoint(90.0); // Middle position
    pid.enableOutputFilter(0.1);
    pid.setTuningDuration(20000); // 20 seconds for servo system
}

void loop()
{
    currentPosition = map(analogRead(POSITION_SENSOR_PIN), 0, 1023, 0, 180);

    pid.update(currentPosition);
    analogWrite(SERVO_PIN, pid.getOutput());

    printData();
    delay(SAMPLE_TIME_MS);
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
