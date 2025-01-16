#include <AutoTunePID.h>

// Pin definitions
const int inputPin = A0; // Analog input pin for reading the sensor value
const int outputPin = 9; // PWM output pin for controlling the actuator

// PID parameters
float setpoint = 100.0; // Desired setpoint
AutoTunePID pid(0, 255, TuningMethod::ZieglerNichols); // Create PID controller with minOutput=0, maxOutput=255, and Ziegler-Nichols tuning method

void setup()
{
    Serial.begin(9600); // Initialize serial communication
    pinMode(inputPin, INPUT); // Set input pin as input
    pinMode(outputPin, OUTPUT); // Set output pin as output

    // Configure PID controller
    pid.setSetpoint(setpoint); // Set the desired setpoint
    pid.setOscillationMode(OscillationMode::Half); // Set oscillation mode to Half (default steps = 20)
    pid.setOperationalMode(OperationalMode::Tune); // Start in Tune mode for auto-tuning
}

void loop()
{
    // Read the current input from the sensor
    float currentInput = analogRead(inputPin);

    // Update the PID controller with the current input
    pid.update(currentInput);

    // Get the computed output from the PID controller
    float output = pid.getOutput();

    // Apply the output to the actuator (e.g., PWM signal to a motor or heater)
    analogWrite(outputPin, output);

    // Print debugging information to the serial monitor
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
    }
    Serial.println();

    // Small delay to maintain a consistent sample time
    delay(100);
}
