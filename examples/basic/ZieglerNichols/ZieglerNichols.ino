#include <AutoTunePID.h>

// Pin definitions
const int inputPin = A0; // Analog input pin for sensor
const int outputPin = 9; // PWM output pin for actuator

// PID parameters
float setpoint = 100.0; // Desired setpoint
AutoTunePID pid(0, 255, TuningMethod::ZieglerNichols); // Min output, max output, tuning method

void setup()
{
    Serial.begin(9600);
    pinMode(inputPin, INPUT);
    pinMode(outputPin, OUTPUT);

    pid.setSetpoint(setpoint);
    pid.setDataPointSize(25); // Set buffer size for tuning
}

void loop()
{
    float currentInput = analogRead(inputPin); // Read sensor input
    pid.update(currentInput); // Update PID controller

    float output = pid.getOutput(); // Get PID output
    analogWrite(outputPin, output); // Write output to actuator

    // Print tuning results
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

    delay(100); // Sample time
}
