/**
 * @file CascadePressureFlow.ino
 * @brief Industrial example demonstrating Cascade PID Control.
 * @details In this system, two PID controllers work together:
 * 1. Master PID (Pressure): Monitors pipe pressure and calculates the required Flow Rate.
 * 2. Slave PID (Flow): Monitors actual flow and calculates the Motor PWM to achieve the master's target.
 * Cascade control provides much faster response to supply disturbances.
 */

#include <AutoTunePID.h>

using namespace atp;

// --- Hardware Configuration ---
const int PRESSURE_SENSOR_PIN = A0;
const int FLOW_SENSOR_PIN = A1;
const int PUMP_PWM_PIN = 9;

// --- System Setpoints ---
float targetPressure = 50.0f; // Target pressure in PSI

// --- PID Controller Instances ---

/** 
 * Master PID: Pressure Control
 * Input: PSI, Output: Target Liters/min (LPM)
 * Range: 0.0 to 100.0 LPM
 */
AutoTunePID pressureMaster(0.0f, 100.0f, TuningMethod::IMC);

/**
 * Slave PID: Flow Control
 * Input: LPM, Output: Pump PWM (0-255)
 * Range: 0.0 to 255.0 PWM
 */
AutoTunePID flowSlave(0.0f, 255.0f, TuningMethod::CohenCoon);

void setup() {
    Serial.begin(115200);

    // Initial Master Setup
    pressureMaster.setSetpoint(targetPressure);
    pressureMaster.enableInputFilter(0.1f);  // Filter pressure noise
    pressureMaster.enableAntiWindup(true, 0.8f);

    // Initial Slave Setup
    // Setpoint will be updated dynamically by the master
    flowSlave.enableInputFilter(0.2f);   // Flow sensors are usually noisier
    flowSlave.enableAntiWindup(true, 0.9f);

    // Note: In a real system, you would tune the Slave (Inner) loop first,
    // then tune the Master (Outer) loop while the slave is running.
    flowSlave.setOperationalMode(OperationalMode::Normal);
    pressureMaster.setOperationalMode(OperationalMode::Normal);

    Serial.println("--- Cascade Pressure/Flow Controller Initialized ---");
}

void loop() {
    static uint32_t lastUpdate = 0U;
    uint32_t now = millis();

    // Standard industrial 100ms control cycle
    if ((now - lastUpdate) >= 100U) {
        lastUpdate = now;

        // 1. Read Sensors
        float currentPressure = static_cast<float>(analogRead(PRESSURE_SENSOR_PIN)) * (100.0f / 1023.0f);
        float currentFlow = static_cast<float>(analogRead(FLOW_SENSOR_PIN)) * (150.0f / 1023.0f);

        // 2. Master Loop (Pressure -> Target Flow)
        pressureMaster.update(currentPressure);
        float targetFlow = pressureMaster.getOutput();

        // 3. Update Slave Setpoint
        flowSlave.setSetpoint(targetFlow);

        // 4. Slave Loop (Flow -> Pump PWM)
        flowSlave.update(currentFlow);
        float pumpPWM = flowSlave.getOutput();

        // 5. Apply to Hardware
        analogWrite(PUMP_PWM_PIN, static_cast<int>(pumpPWM));

        // 6. Monitoring
        Serial.print("Pres: "); Serial.print(currentPressure, 1);
        Serial.print(" / "); Serial.print(targetPressure, 1);
        Serial.print(" PSI | Flow: "); Serial.print(currentFlow, 1);
        Serial.print(" / "); Serial.print(targetFlow, 1);
        Serial.print(" LPM | PWM: "); Serial.println(pumpPWM, 0);
    }
}
