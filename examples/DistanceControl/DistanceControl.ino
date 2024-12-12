/*
 * This example demonstrates how to use the AutoTunePID library to maintain a target distance.
 *
 * - An ultrasonic sensor (e.g., HC-SR04) is used to measure the distance from an object.
 * - The target distance (setpoint) is specified in the code.
 * - The AutoTunePID library dynamically tunes the PID values for precise control.
 * - The PID output adjusts the speed of a motor using PWM to maintain the desired distance.
 * - This setup is ideal for obstacle avoidance robots, conveyor belt systems, or automated gates.
 */

#include <AutoTunePID.h>

// Define pins
#define TRIGGER_PIN 7
#define ECHO_PIN 6
#define MOTOR_PIN 3

// Create PID instance
AutoTunePID pid(0, 255, 10000);

void setup() {
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(MOTOR_PIN, OUTPUT);
    Serial.begin(9600);

    // Set PID tunings
    pid.setTunings(1.5, 0.3, 0.05); // Kp, Ki, Kd
}

float getDistance() {
    // Send ultrasonic pulse
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    // Measure the time for the echo
    long duration = pulseIn(ECHO_PIN, HIGH);

    // Convert to distance (cm)
    return (duration * 0.034 / 2);
}

void loop() {
    // Measure current distance
    float currentDistance = getDistance();

    // Compute PID output to control motor
    float output = pid.compute(currentDistance, 50.0); // Target distance: 50cm

    // Send the output to the motor
    analogWrite(MOTOR_PIN, output);

    // Debugging info
    Serial.print("Distance: ");
    Serial.print(currentDistance);
    Serial.print(" cm | PID Output: ");
    Serial.println(output);

    delay(100); // Sampling delay
}
