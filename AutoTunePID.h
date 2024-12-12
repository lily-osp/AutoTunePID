#ifndef AutoTunePID_H
#define AutoTunePID_H

#include <Arduino.h>

class AutoTunePID {
public:
    AutoTunePID(float maxOut, float minOut, unsigned long tuneDuration);

    void autoTune(float currentReading, float setpoint);
    float compute(float currentReading, float setpoint);
    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float maxOut, float minOut);
    void reset();
    void setMode(bool isAuto);
    void setSetpointRamp(float rampRate);
    void saveToEEPROM(int address);
    void loadFromEEPROM(int address);

private:
    // PID constants
    float Kp, Ki, Kd;

    // Output limits
    float maxOutput, minOutput;

    // Auto-tuning variables
    bool tuning, isAutoMode;
    float Ku, Tu;
    unsigned long tuningStartTime, tuningDuration;
    float maxOutputDuringTuning, minOutputDuringTuning;

    // PID variables
    float integral, lastError, setpointRampRate, setpoint;
    float lastOutput;
    unsigned long lastTime;

    // Prevent integral windup
    void clampIntegral();
};

#endif
