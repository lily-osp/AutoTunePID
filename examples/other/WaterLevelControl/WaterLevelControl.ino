/*
 * Auto-Tune Water Level Control
 *
 * - An ultrasonic sensor measures the water level in a tank.
 * - A water pump adjusts to maintain the target water level.
 * - AutoTunePID dynamically adjusts the PID parameters for changing conditions.
 * - Applications include irrigation systems, water tanks, or flood prevention.
 */

#include <AutoTunePID.h>

// Define pins
#define triggerPin 7
#define echoPin 6
#define pumpPin 3

// Create PID instance
AutoTunePID pid(255, 0, 15000); // PWM range: 0-255, longer auto-tuning period for stability

void setup() {
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(pumpPin, OUTPUT);
    Serial.begin(9600);

    // Enable auto-tuning
    pid.enableAutoTune(true);
    pid.setAutoTuneRange(5); // Auto-tune for errors greater than 5 cm
    pid.setTargetRange(2);  // Maintain level within +/- 2 cm of the target
}

float measureWaterLevel() {
    // Trigger ultrasonic pulse
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    // Measure the time for the echo
    long duration = pulseIn(echoPin, HIGH);

    // Convert to water level in cm (adjust as needed for your setup)
    return (duration * 0.034 / 2);
}

void loop() {
    // Measure current water level
    float currentLevel = measureWaterLevel();

    // Compute PID output to adjust pump speed
    float output = pid.compute(currentLevel, 30.0); // Target level: 30 cm

    // Control water pump
    analogWrite(pumpPin, constrain(output, 0, 255));

    // Debugging info
    Serial.print("Water Level: ");
    Serial.print(currentLevel);
    Serial.print(" cm | Pump PWM: ");
    Serial.println(output);

    delay(100); // Sampling delay
}
