#ifndef AUTOTUNEPID_H
#define AUTOTUNEPID_H

#include <Arduino.h>

// Enumeration for different tuning methods
enum class TuningMethod {
    ZieglerNichols, // Ziegler-Nichols tuning method
    CohenCoon, // Cohen-Coon tuning method
    IMC, // Internal Model Control tuning method
    TyreusLuyben, // Tyreus-Luyben tuning method
    LambdaTuning, // Lambda Tuning (CLD) method
    Manual // Manual tuning method
};

// Backward compatibility for tuning methods
constexpr auto ZieglerNichols = TuningMethod::ZieglerNichols;
constexpr auto CohenCoon = TuningMethod::CohenCoon;
constexpr auto IMC = TuningMethod::IMC;
constexpr auto TyreusLuyben = TuningMethod::TyreusLuyben;
constexpr auto LambdaTuning = TuningMethod::LambdaTuning;
constexpr auto Manual = TuningMethod::Manual;

// Enumeration for operational modes
enum class OperationalMode {
    Normal, // Normal PID operation
    Reverse, // Reverse PID operation (e.g., for cooling systems)
    Manual, // Manual mode: direct output control, bypass PID calculations
    Override, // Override mode: emergency override with fixed output value
    Track, // Track mode: output follows a reference signal
    Hold, // Hold the PID calculations to save resources
    Preserve, // Preserve mode: minimal calculations, keep the system responsive
    Tune, // Tune mode: perform auto-tuning to get Tu and Ku
    Auto // Auto mode: automatically determine the best operational mode
};

// Enumeration for oscillation modes
enum class OscillationMode {
    Normal, // Full oscillation (MaxOutput - MinOutput)
    Half, // Half oscillation (1/2 MaxOutput - 1/2 MinOutput)
    Mild // Mild oscillation (1/4 MaxOutput - 1/4 MinOutput)
};

class AutoTunePID {
public:
    // Constructor to initialize the PID controller with min/max output and tuning method
    AutoTunePID(float minOutput, float maxOutput, TuningMethod method = TuningMethod::ZieglerNichols);

    // Configuration methods
    void setSetpoint(float setpoint); // Set the desired setpoint
    void setTuningMethod(TuningMethod method); // Set the tuning method
    void setManualGains(float kp, float ki, float kd); // Set manual PID gains
    void enableInputFilter(float alpha); // Enable input filtering with a given alpha value
    void enableOutputFilter(float alpha); // Enable output filtering with a given alpha value
    void enableAntiWindup(bool enable, float threshold = 0.8f); // Enable/disable anti-windup with optional threshold
    void setOperationalMode(OperationalMode mode); // Set the operational mode
    void setManualOutput(float output); // Set manual output value (0-100% for Manual mode)
    void setOverrideOutput(float output); // Set override output value
    void setTrackReference(float reference); // Set track reference signal
    void setOscillationMode(OscillationMode mode); // Set the oscillation mode for auto-tuning
    void setOscillationSteps(int steps); // Set the number of oscillation steps for auto-tuning
    void setLambda(float lambda); // Set the lambda parameter for Lambda Tuning (CLD)

    // Runtime methods
    void update(float currentInput); // Update the PID controller with the current input
    float getOutput() const { return _output; } // Get the current output value
    float getKp() const { return _kp; } // Get the proportional gain (Kp)
    float getKi() const { return _ki; } // Get the integral gain (Ki)
    float getKd() const { return _kd; } // Get the derivative gain (Kd)
    float getKu() const { return _ultimateGain; } // Get the ultimate gain (Ku)
    float getTu() const { return _oscillationPeriod; } // Get the oscillation period (Tu)
    float getSetpoint() const { return _setpoint; } // Get the current setpoint
    OperationalMode getOperationalMode() const { return _operationalMode; } // Get the current operational mode
    float getLambda() const { return _lambda; } // Get the lambda parameter for Lambda Tuning (CLD)

private:
    // PID computation
    void computePID(); // Compute the PID output
    void applyAntiWindup(); // Apply anti-windup to prevent integral windup
    float computeFilteredValue(float input, float& filteredValue, float alpha) const; // Compute filtered value using exponential moving average

    // Autotuning methods
    void performAutoTune(float currentInput); // Perform auto-tuning based on the current input
    void calculateZieglerNicholsGains(); // Calculate PID gains using Ziegler-Nichols method
    void calculateCohenCoonGains(); // Calculate PID gains using Cohen-Coon method
    void calculateIMCGains(); // Calculate PID gains using IMC method
    void calculateTyreusLuybenGains(); // Calculate PID gains using Tyreus-Luyben method
    void calculateLambdaTuningGains(); // Calculate PID gains using Lambda Tuning (CLD) method

    // Configuration
    const float _minOutput; // Minimum output value
    const float _maxOutput; // Maximum output value
    TuningMethod _method; // Current tuning method
    OperationalMode _operationalMode; // Current operational mode
    OscillationMode _oscillationMode; // Current oscillation mode for auto-tuning
    int _oscillationSteps; // Number of oscillation steps for auto-tuning
    float _setpoint; // Desired setpoint
    float _lambda; // Lambda parameter for Lambda Tuning (CLD)

    // Manual, Override, and Track mode parameters
    float _manualOutput; // Manual output value (0-100%)
    float _overrideOutput; // Override output value
    float _trackReference; // Track reference signal

    // PID parameters
    float _kp, _ki, _kd; // Proportional, integral, and derivative gains
    float _error, _previousError; // Current and previous error values
    float _integral, _derivative; // Integral and derivative terms
    float _output; // Current output value
    float _input; // Current input value (e.g., sensor reading)

    // Anti-windup
    bool _antiWindupEnabled; // Flag to enable/disable anti-windup
    float _integralWindupThreshold; // Threshold for integral windup

    // Autotuning parameters
    unsigned long _lastUpdate; // Timestamp of the last update
    float _ultimateGain; // Ultimate gain (Ku)
    float _oscillationPeriod; // Oscillation period (Tu)

    // Additional parameters for advanced tuning
    float _processTimeConstant; // Process time constant (T)
    float _deadTime; // Dead time (L)
    float _integralTime; // Integral time (Ti)
    float _derivativeTime; // Derivative time (Td)

    // Filtering
    bool _inputFilterEnabled; // Flag to enable/disable input filtering
    bool _outputFilterEnabled; // Flag to enable/disable output filtering
    float _inputFilteredValue; // Filtered input value
    float _outputFilteredValue; // Filtered output value
    float _inputFilterAlpha; // Alpha value for input filtering
    float _outputFilterAlpha; // Alpha value for output filtering
};

#endif
