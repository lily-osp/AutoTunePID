/**
 * @file BoosterMotorController.ino
 * @brief Professional example for an electronic motor speed booster system.
 * @details This example demonstrates using the AutoTunePID library to control a high-speed motor.
 * It utilizes input filtering for noisy encoder data and anti-windup for physical motor limits.
 * The system starts in 'Tune' mode to identify optimal Cohen-Coon gains.
 */

#include <AutoTunePID.h>

// Use the atp namespace for AUTOSAR compliance
using namespace atp;

// --- Hardware Configuration ---
const int ENCODER_PIN = 2;   // Interrupt-capable pin for speed sensing
const int MOTOR_PWM_PIN = 9; // PWM output for motor speed
const int MOTOR_DIR_PIN = 8; // Digital output for motor direction

// --- System Parameters ---
float targetRPM = 1500.0f;    // Desired motor speed
volatile uint32_t pulses = 0U; // Counter for encoder pulses

// --- PID Controller Instance ---
// Range: 0.0 (Stop) to 255.0 (Full Booster Power)
// Method: CohenCoon (Excellent for systems with physical lag/inertia like motors)
AutoTunePID booster(0.0f, 255.0f, TuningMethod::CohenCoon);

/**
 * @brief ISR for encoder pulses.
 */
void countPulse() {
    pulses++;
}

void setup() {
    Serial.begin(115200);

    // Pin Setup
    pinMode(ENCODER_PIN, INPUT_PULLUP);
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countPulse, RISING);

    // Direction initialization
    digitalWrite(MOTOR_DIR_PIN, HIGH);

    // PID Configuration
    booster.setSetpoint(targetRPM);
    
    // Enable input filter (alpha=0.2) to smooth out high-frequency encoder noise
    booster.enableInputFilter(0.2f);
    
    // Enable anti-windup at 90% threshold to prevent integral runaway during acceleration
    booster.enableAntiWindup(true, 0.9f);
    
    // Set tuning mode to 'Half' scope (oscillate around center power for safety)
    booster.setOscillationMode(OscillationMode::Half);
    
    // Start in TUNE mode to find optimal gains for the current motor/load
    booster.setOperationalMode(OperationalMode::Tune);

    Serial.println("--- Booster Motor Controller Initialized ---");
    Serial.println("Starting Auto-Tuning sequence...");
}

void loop() {
    static uint32_t lastMillis = 0U;
    uint32_t now = millis();

    // Update speed measurement every 100ms
    if ((now - lastMillis) >= 100U) {
        // Atomic read of pulses to prevent race conditions on 8-bit AVR
        noInterrupts();
        uint32_t currentPulses = pulses;
        pulses = 0U;
        interrupts();

        // Calculate RPM: (currentPulses in 100ms * 10 * 60) / pulses_per_rev
        // Assuming 20 pulses per revolution for this example
        float currentRPM = static_cast<float>(currentPulses * 600) / 20.0f;
        lastMillis = now;

        // Update PID controller
        booster.update(currentRPM);

        // Apply booster output
        analogWrite(MOTOR_PWM_PIN, static_cast<int>(booster.getOutput()));

        // Monitor Progress
        Serial.print("RPM: ");
        Serial.print(currentRPM);
        Serial.print(" | Booster Output: ");
        Serial.print(booster.getOutput());
        Serial.print(" | Mode: ");

        switch (booster.getOperationalMode()) {
            case OperationalMode::Tune:
                Serial.println("TUNING...");
                break;
            case OperationalMode::Normal:
                Serial.println("STABLE");
                break;
            case OperationalMode::Manual:
                Serial.println("MANUAL");
                break;
            case OperationalMode::Override:
                Serial.println("OVERRIDE!");
                break;
            default:
                Serial.println("UNKNOWN");
                break;
        }

        // Check if tuning finished
        static bool tuningDone = false;
        if (!tuningDone && booster.getOperationalMode() == OperationalMode::Normal) {
            tuningDone = true;
            Serial.println("\n>>> AUTO-TUNING COMPLETE <<<");
            Serial.print("New Gains -> Kp: "); Serial.print(booster.getKp());
            Serial.print(" | Ki: "); Serial.print(booster.getKi());
            Serial.print(" | Kd: "); Serial.println(booster.getKd());
            Serial.println("Switching to active booster control.\n");
        }
    }
}
