#include "AutoTunePID.h"
#include <EEPROM.h>

AutoTunePID::AutoTunePID(float maxOut, float minOut, unsigned long tuneDuration) {
    maxOutput = maxOut;
    minOutput = minOut;
    tuningDuration = tuneDuration;

    // Initialize variables
    tuning = true;
    isAutoMode = true; // Default to auto mode
    Kp = Ki = Kd = 0;
    integral = 0;
    lastError = 0;
    lastTime = millis();
    maxOutputDuringTuning = minOutputDuringTuning = 0;
    setpointRampRate = 0;
    setpoint = 0;
}

void AutoTunePID::autoTune(float currentReading, float setpoint) {
    if (!tuning) return;

    float error = setpoint - currentReading;
    float output = error;

    if (output > maxOutputDuringTuning) maxOutputDuringTuning = output;
    if (output < minOutputDuringTuning) minOutputDuringTuning = output;

    if (millis() - tuningStartTime > tuningDuration) {
        tuning = false;
        Ku = 4 * (maxOutputDuringTuning - minOutputDuringTuning) /
             (3.14159 * (maxOutputDuringTuning + minOutputDuringTuning));
        Tu = tuningDuration / 1000.0;

        // Ziegler-Nichols tuning
        Kp = 0.6 * Ku;
        Ki = 2 * Kp / Tu;
        Kd = Kp * Tu / 8;
    }
}

float AutoTunePID::compute(float currentReading, float setpoint) {
    if (!isAutoMode) return lastOutput;

    // Smoothly ramp setpoint
    if (setpointRampRate > 0 && setpoint != this->setpoint) {
        float delta = setpoint - this->setpoint;
        if (abs(delta) > setpointRampRate) {
            this->setpoint += (delta > 0 ? setpointRampRate : -setpointRampRate);
        } else {
            this->setpoint = setpoint;
        }
    } else {
        this->setpoint = setpoint;
    }

    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; // Seconds
    lastTime = currentTime;

    if (dt <= 0) return lastOutput; // Avoid division by zero

    float error = this->setpoint - currentReading;

    // Proportional term
    float P = Kp * error;

    // Integral term
    integral += error * dt;
    clampIntegral();
    float I = Ki * integral;

    // Derivative term
    float derivative = (error - lastError) / dt;
    float D = Kd * derivative;

    lastError = error;

    // PID output
    float output = P + I + D;

    // Clamp output
    if (output > maxOutput) output = maxOutput;
    if (output < minOutput) output = minOutput;

    lastOutput = output;
    return output;
}

void AutoTunePID::setTunings(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void AutoTunePID::setOutputLimits(float maxOut, float minOut) {
    maxOutput = maxOut;
    minOutput = minOut;
}

void AutoTunePID::reset() {
    integral = 0;
    lastError = 0;
}

void AutoTunePID::setMode(bool isAuto) {
    isAutoMode = isAuto;
    if (!isAutoMode) reset();
}

void AutoTunePID::setSetpointRamp(float rampRate) {
    setpointRampRate = rampRate;
}

void AutoTunePID::saveToEEPROM(int address) {
    EEPROM.put(address, Kp);
    EEPROM.put(address + sizeof(Kp), Ki);
    EEPROM.put(address + sizeof(Kp) + sizeof(Ki), Kd);
}

void AutoTunePID::loadFromEEPROM(int address) {
    EEPROM.get(address, Kp);
    EEPROM.get(address + sizeof(Kp), Ki);
    EEPROM.get(address + sizeof(Kp) + sizeof(Ki), Kd);
}

void AutoTunePID::clampIntegral() {
    if (integral > maxOutput / Ki) integral = maxOutput / Ki;
    if (integral < minOutput / Ki) integral = minOutput / Ki;
}
