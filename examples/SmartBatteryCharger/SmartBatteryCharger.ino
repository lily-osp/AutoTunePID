/**
 * @file SmartBatteryCharger.ino
 * @brief Professional example for a Constant Current / Constant Voltage (CC/CV) battery charger.
 * @details This example demonstrates advanced mode switching and safety logic:
 * 1. Constant Current (CC) Phase: PID controls current until target voltage is reached.
 * 2. Constant Voltage (CV) Phase: PID maintains voltage while current naturally decays.
 * 3. Automatic Safety: Uses Override mode for emergency shutoff and Track mode for soft-start.
 */

#include <AutoTunePID.h>

using namespace atp;

// --- Hardware Configuration ---
const int VOLTAGE_SENSE_PIN = A0;
const int CURRENT_SENSE_PIN = A1;
const int CHARGER_PWM_PIN = 9;

// --- Battery Parameters ---
const float TARGET_VOLTAGE = 14.4f; // Bulk/Absorption voltage (e.g., 12V Lead Acid)
const float CHARGE_CURRENT_LIMIT = 5.0f; // Max charge current in Amps
const float TERMINATION_CURRENT = 0.5f; // Charge complete threshold

// --- PID Controller Instances ---
// Shared Output Range: 0.0 to 255.0 (Charger PWM)
AutoTunePID ccPID(0.0f, 255.0f, TuningMethod::TyreusLuyben); // For Current control
AutoTunePID cvPID(0.0f, 255.0f, TuningMethod::IMC);          // For Voltage control

// --- Charger State ---
enum class ChargerState : uint8_t {
    IDLE,
    SOFT_START,
    CC_PHASE,
    CV_PHASE,
    CHARGED,
    FAULT
};

ChargerState currentState = ChargerState::IDLE;

void setup() {
    Serial.begin(115200);

    // Initial Configuration
    ccPID.setSetpoint(CHARGE_CURRENT_LIMIT);
    ccPID.enableAntiWindup(true, 0.85f);
    
    cvPID.setSetpoint(TARGET_VOLTAGE);
    cvPID.enableAntiWindup(true, 0.9f);
    
    // Safety first: start in Override mode (0 output)
    ccPID.setOverrideOutput(0.0f);
    ccPID.setOperationalMode(OperationalMode::Override);

    Serial.println("--- Smart CC/CV Battery Charger Initialized ---");
}

void loop() {
    static uint32_t lastUpdate = 0U;
    uint32_t now = millis();

    // 100ms precise control loop
    if ((now - lastUpdate) >= 100U) {
        lastUpdate = now;

        // 1. Read Sensors
        float battVoltage = static_cast<float>(analogRead(VOLTAGE_SENSE_PIN)) * (20.0f / 1023.0f); // 0-20V range
        float chargeCurrent = static_cast<float>(analogRead(CURRENT_SENSE_PIN)) * (10.0f / 1023.0f); // 0-10A range

        float chargerOutput = 0.0f;

        // 2. State Machine Logic
        switch (currentState) {
            case ChargerState::IDLE:
                if (battVoltage > 2.0f && battVoltage < TARGET_VOLTAGE) {
                    currentState = ChargerState::CC_PHASE;
                    ccPID.setOperationalMode(OperationalMode::Normal);
                    Serial.println("Entering CC Phase (Constant Current)");
                }
                break;

            case ChargerState::CC_PHASE:
                ccPID.update(chargeCurrent);
                chargerOutput = ccPID.getOutput();
                
                // Switch to CV when voltage reached
                if (battVoltage >= TARGET_VOLTAGE) {
                    currentState = ChargerState::CV_PHASE;
                    // Transfer current PWM to CV PID to prevent sudden jumps (Smooth Handover)
                    cvPID.setManualOutput((chargerOutput / 255.0f) * 100.0f);
                    cvPID.setOperationalMode(OperationalMode::Normal);
                    Serial.println("Entering CV Phase (Constant Voltage)");
                }
                break;

            case ChargerState::CV_PHASE:
                cvPID.update(battVoltage);
                chargerOutput = cvPID.getOutput();

                // Termination condition
                if (chargeCurrent < TERMINATION_CURRENT) {
                    currentState = ChargerState::CHARGED;
                    cvPID.setOperationalMode(OperationalMode::Override);
                    cvPID.setOverrideOutput(0.0f);
                    Serial.println("Battery Fully Charged!");
                }
                break;

            case ChargerState::CHARGED:
                chargerOutput = 0.0f;
                break;

            default:
                chargerOutput = 0.0f;
                break;
        }

        // 3. Hardware Safety & Output
        if (battVoltage > 16.0f) { // Over-voltage fault
            currentState = ChargerState::FAULT;
            chargerOutput = 0.0f;
            Serial.println("CRITICAL FAULT: OVER-VOLTAGE!");
        }

        analogWrite(CHARGER_PWM_PIN, static_cast<int>(chargerOutput));

        // 4. Monitoring
        Serial.print("V: "); Serial.print(battVoltage, 2);
        Serial.print("V | I: "); Serial.print(chargeCurrent, 2);
        Serial.print("A | PWM: "); Serial.print(chargerOutput, 0);
        Serial.print(" | State: "); Serial.println(static_cast<int>(currentState));
    }
}
