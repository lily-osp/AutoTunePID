/**
 * @file DeterministicRTOS.ino
 * @brief Advanced example demonstrating deterministic execution using the Explicit Delta-Time API.
 * @details This example shows how to use the `update(input, dt)` method inside a hardware 
 * timer interrupt (or an RTOS task) to guarantee execution exactly every 50ms, bypassing
 * the standard Arduino `loop()` and `millis()` timing which can suffer from jitter.
 */

#include <AutoTunePID.h>

using namespace atp;

// --- Hardware Configuration ---
const int SENSOR_PIN = A0;
const int ACTUATOR_PIN = 9;

// --- PID Controller Instance ---
AutoTunePID precisionPID(0.0f, 255.0f, TuningMethod::ZieglerNichols);

// --- State Variables ---
volatile float currentSensorValue = 0.0f;
volatile float currentOutput = 0.0f;
volatile bool newOutputAvailable = false;

// 50ms interval (0.05 seconds)
const float DT_SECONDS = 0.05f; 

void setup() {
    Serial.begin(115200);
    pinMode(ACTUATOR_PIN, OUTPUT);

    precisionPID.setSetpoint(100.0f);
    
    // Disable the internal filter as we have a very fast, strict loop
    // and might want to handle filtering externally.
    
    // Setup Timer1 to fire an interrupt every 50ms (AVR specific example)
    // Note: Timer setup varies wildly between AVR, ESP32, STM32. 
    // This is a generic AVR implementation.
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    
    // Compare match register for 50ms (20Hz) at 16MHz with 256 prescaler
    OCR1A = 3125; 
    
    // Turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS12 bit for 256 prescaler
    TCCR1B |= (1 << CS12);  
    // Enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    interrupts();

    Serial.println("--- Deterministic RTOS/ISR Controller Started ---");
}

// Timer1 Interrupt Service Routine
// Fires exactly every 50ms
ISR(TIMER1_COMPA_vect) {
    // 1. Fast read
    int rawVal = analogRead(SENSOR_PIN);
    currentSensorValue = static_cast<float>(rawVal) * (200.0f / 1023.0f);

    // 2. Deterministic PID Update
    // We pass our known, strict DT_SECONDS directly into the math core.
    precisionPID.update(currentSensorValue, DT_SECONDS);
    
    currentOutput = precisionPID.getOutput();
    newOutputAvailable = true;
}

void loop() {
    // The main loop is completely free to handle slow tasks like
    // Serial communication, displays, or network traffic without
    // causing any jitter to the PID control.

    if (newOutputAvailable) {
        // Atomic read not strictly needed for single float on some archs, 
        // but good practice.
        noInterrupts();
        float outputToApply = currentOutput;
        float sensorToPrint = currentSensorValue;
        newOutputAvailable = false;
        interrupts();

        analogWrite(ACTUATOR_PIN, static_cast<int>(outputToApply));

        Serial.print("Sensor: "); Serial.print(sensorToPrint);
        Serial.print(" | PWM: "); Serial.println(outputToApply);
    }
}
