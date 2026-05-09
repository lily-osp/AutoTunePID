/**
 * @file DeltaRobotJoint.ino
 * @brief Advanced example for high-precision position control of a Delta Robot joint.
 * @details This example demonstrates using the AutoTunePID library for a geared DC motor.
 * It features position control (in degrees), Cohen-Coon tuning for handling gearbox friction,
 * and strict physical limits for robot safety.
 */

#include <AutoTunePID.h>

// Use the atp namespace for AUTOSAR compliance
using namespace atp;

// --- Hardware Configuration ---
const int ENCODER_PIN_A = 2;   // Phase A for position tracking
const int ENCODER_PIN_B = 3;   // Phase B for direction tracking
const int MOTOR_PWM_PIN = 10;  // PWM output for torque/speed
const int MOTOR_DIR_PIN = 7;   // Joint direction

// --- Robot Physical Constraints ---
const float MIN_ANGLE = -45.0f; // Minimum arm angle in degrees
const float MAX_ANGLE = 90.0f;  // Maximum arm angle in degrees
const float PULSES_PER_DEGREE = 14.22f; // Example: 512 PPR encoder * 10:1 gearbox / 360

// --- System State ---
volatile int32_t encoderTicks = 0;
float targetAngle = 45.0f; // Target joint position

// --- PID Controller Instance ---
// Output Range: -255 to 255 (Bidirectional PWM)
// Method: CohenCoon (Excellent for high-inertia/friction robotic joints)
AutoTunePID jointPID(-255.0f, 255.0f, TuningMethod::CohenCoon);

/**
 * @brief ISR for encoder tracking.
 */
void handleEncoder() {
    if (digitalRead(ENCODER_PIN_B) == HIGH) {
        encoderTicks++;
    } else {
        encoderTicks--;
    }
}

void setup() {
    Serial.begin(115200);

    // Hardware Setup
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), handleEncoder, RISING);

    // PID Configuration
    jointPID.setSetpoint(targetAngle);
    
    // Smooth out noisy encoder pulses from fast movements
    jointPID.enableInputFilter(0.15f);
    
    // Use 85% anti-windup to allow quick deceleration without overshoot
    jointPID.enableAntiWindup(true, 0.85f);
    
    // Safety: oscillate only within +/- 15 degrees during tuning
    jointPID.setOscillationMode(OscillationMode::Mild);
    
    // Start tuning to find the best gains for the arm's weight and friction
    jointPID.setOperationalMode(OperationalMode::Tune);

    Serial.println(">>> Delta Robot Joint Controller <<<");
    Serial.println("Initiating safe auto-tuning sequence...");
}

void loop() {
    static uint32_t lastLoop = 0U;
    uint32_t now = millis();

    // Standard 50ms control loop for robotics (100ms is standard for this lib)
    if ((now - lastLoop) >= 100U) {
        lastLoop = now;

        // 1. Calculate current joint position
        float currentAngle = static_cast<float>(encoderTicks) / PULSES_PER_DEGREE;

        // 2. Update PID with current position
        jointPID.update(currentAngle);

        // 3. Get signed output and apply to motor driver
        float pidOutput = jointPID.getOutput();
        
        // Handle bidirectional motor driver
        if (pidOutput >= 0.0f) {
            digitalWrite(MOTOR_DIR_PIN, HIGH);
            analogWrite(MOTOR_PWM_PIN, static_cast<int>(pidOutput));
        } else {
            digitalWrite(MOTOR_DIR_PIN, LOW);
            analogWrite(MOTOR_PWM_PIN, static_cast<int>(fabsf(pidOutput)));
        }

        // 4. Telemetry
        Serial.print("Angle: "); Serial.print(currentAngle, 2);
        Serial.print("° | Target: "); Serial.print(targetAngle, 1);
        Serial.print("° | Power: "); Serial.print(pidOutput);
        
        if (jointPID.getOperationalMode() == OperationalMode::Tune) {
            Serial.println(" [TUNING]");
        } else {
            Serial.println(" [LOCKED]");
        }

        // 5. Completion Logic
        static bool tuningReported = false;
        if (!tuningReported && jointPID.getOperationalMode() == OperationalMode::Normal) {
            tuningReported = true;
            Serial.println("\n--- Robot Calibration Successful ---");
            Serial.print("Optimal P: "); Serial.println(jointPID.getKp(), 4);
            Serial.print("Optimal I: "); Serial.println(jointPID.getKi(), 4);
            Serial.print("Optimal D: "); Serial.println(jointPID.getKd(), 4);
            Serial.println("Arm is ready for coordinated movement.\n");
        }
    }
    
    // Handle serial commands for testing (non-blocking)
    if (Serial.available()) {
        float newTarget = Serial.parseFloat();
        if (newTarget >= MIN_ANGLE && newTarget <= MAX_ANGLE) {
            targetAngle = newTarget;
            jointPID.setSetpoint(targetAngle);
            Serial.print("Moving arm to: "); Serial.println(targetAngle);
        }
    }
}
