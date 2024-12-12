/*
 * Auto-Tune Servo Angle Control
 *
 * - A potentiometer is used to set the target angle (setpoint).
 * - A servo motor adjusts to maintain the target angle.
 * - AutoTunePID dynamically tunes PID values to optimize servo response.
 * - Ideal for robotic arms, gimbal stabilization, or precise positioning systems.
 */

#include <AutoTunePID.h>
#include <Servo.h>

// Define pins
#define potPin A0
#define servoPin 9

// Create PID instance
AutoTunePID pid(180, 0, 10000); // Servo angle range: 0-180
Servo servo;

void setup() {
    servo.attach(servoPin);
    Serial.begin(9600);

    // Enable auto-tuning
    pid.enableAutoTune(true);
    pid.setAutoTuneRange(10); // Auto-tune when the error exceeds 10 degrees
}

void loop() {
    // Read target angle from potentiometer (scaled to 0-180)
    float targetAngle = map(analogRead(potPin), 0, 1023, 0, 180);

    // Simulated current angle feedback (replace with actual sensor for real systems)
    static float currentAngle = 90; // Start at mid-range
    currentAngle += random(-1, 2);  // Simulate small random deviations

    // Compute PID output
    float output = pid.compute(currentAngle, targetAngle);

    // Move servo based on PID output
    currentAngle = constrain(currentAngle + output, 0, 180);
    servo.write(currentAngle);

    // Debugging info
    Serial.print("Target Angle: ");
    Serial.print(targetAngle);
    Serial.print(" | Current Angle: ");
    Serial.print(currentAngle);
    Serial.print(" | PID Output: ");
    Serial.println(output);

    delay(50);
}
