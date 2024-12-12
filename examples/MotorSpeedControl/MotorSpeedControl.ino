/*
 * Auto-Tune DC Motor Speed Control with Noise Filtering
 *
 * - An encoder measures the motor's current speed (RPM).
 * - The AutoTunePID library dynamically tunes the PID values.
 * - A moving average filter reduces noise in the encoder feedback signal.
 * - The PID output controls motor speed using PWM.
 * - Applications include robotic wheels, conveyor belts, or precision equipment.
 */

#include <AutoTunePID.h>

// Define pins
#define motorPin 3
#define encoderPin A0

// Create PID instance
AutoTunePID pid(255, 0, 10000); // PWM range for motor control

// Moving average filter variables
#define FILTER_WINDOW 5
float filterBuffer[FILTER_WINDOW];
int filterIndex = 0;

void setup() {
    pinMode(motorPin, OUTPUT);
    Serial.begin(9600);

    // Enable auto-tuning
    pid.enableAutoTune(true);
    pid.setAutoTuneRange(20); // Auto-tune when speed error exceeds 20 RPM
    pid.setFilter(0.2);       // Use built-in low-pass filter to smooth noisy inputs
}

// Simulate encoder reading (replace with actual encoder code)
float getMotorSpeed() {
    return analogRead(encoderPin) * (5000.0 / 1023.0); // Convert analog input to RPM
}

// Apply a moving average filter to the feedback
float applyMovingAverage(float newValue) {
    filterBuffer[filterIndex] = newValue;
    filterIndex = (filterIndex + 1) % FILTER_WINDOW;

    float sum = 0;
    for (int i = 0; i < FILTER_WINDOW; i++) {
        sum += filterBuffer[i];
    }
    return sum / FILTER_WINDOW;
}

void loop() {
    // Read and filter the motor speed
    float rawSpeed = getMotorSpeed();
    float currentSpeed = applyMovingAverage(rawSpeed);

    // Compute PID output to control motor speed
    float targetSpeed = 1500.0; // Desired speed in RPM
    float output = pid.compute(currentSpeed, targetSpeed);

    // Adjust motor speed
    analogWrite(motorPin, constrain(output, 0, 255));

    // Debugging info
    Serial.print("Filtered Speed: ");
    Serial.print(currentSpeed);
    Serial.print(" RPM | Target Speed: ");
    Serial.print(targetSpeed);
    Serial.print(" | PID Output: ");
    Serial.println(output);

    delay(100); // Sampling delay
}
