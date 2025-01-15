#ifndef AUTOTUNEPID_H
#define AUTOTUNEPID_H

#include <Arduino.h>

// Enumeration for different tuning methods
enum class TuningMethod {
    ZieglerNichols, // Ziegler-Nichols tuning method
    CohenCoon, // Cohen-Coon tuning method
    RelayFeedback, // Relay feedback tuning method
    IMC, // Internal Model Control tuning method
    TyreusLuyben, // Tyreus-Luyben tuning method
    Manual // Manual tuning method
};

// Backward compatibility for tuning methods
constexpr auto ZieglerNichols = TuningMethod::ZieglerNichols;
constexpr auto CohenCoon = TuningMethod::CohenCoon;
constexpr auto RelayFeedback = TuningMethod::RelayFeedback;
constexpr auto IMC = TuningMethod::IMC;
constexpr auto TyreusLuyben = TuningMethod::TyreusLuyben;
constexpr auto Manual = TuningMethod::Manual;

// Structure to store data points for real-time tuning
struct DataPoint {
    float input; // Current input value
    float setpoint; // Desired setpoint
    float output; // Current output value
    unsigned long timestamp; // Timestamp of the data point
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
    void setDataPointSize(int size); // Set the size of the data point buffer
    void enableAntiWindup(bool enable, float threshold = 0.8f); // Enable/disable anti-windup with optional threshold

    // Runtime methods
    void update(float currentInput); // Update the PID controller with the current input
    float getOutput() const { return _output; } // Get the current output value
    float getKp() const { return _kp; } // Get the proportional gain (Kp)
    float getKi() const { return _ki; } // Get the integral gain (Ki)
    float getKd() const { return _kd; } // Get the derivative gain (Kd)
    float getKu() const { return _ultimateGain; } // Get the ultimate gain (Ku)
    float getTu() const { return _oscillationPeriod; } // Get the oscillation period (Tu)
    float getSetpoint() const { return _setpoint; } // Get the current setpoint

private:
    // PID computation
    void computePID(); // Compute the PID output
    void applyAntiWindup(); // Apply anti-windup to prevent integral windup
    float computeFilteredValue(float input, float& filteredValue, float alpha) const; // Compute filtered value using exponential moving average

    // Autotuning methods
    void performAutoTune(float currentInput); // Perform auto-tuning based on the current input
    void calculateZieglerNicholsGains(); // Calculate PID gains using Ziegler-Nichols method
    void calculateCohenCoonGains(); // Calculate PID gains using Cohen-Coon method
    void calculateRelayFeedbackGains(); // Calculate PID gains using Relay Feedback method
    void calculateIMCGains(); // Calculate PID gains using IMC method
    void calculateTyreusLuybenGains(); // Calculate PID gains using Tyreus-Luyben method

    // Configuration
    const float _minOutput; // Minimum output value
    const float _maxOutput; // Maximum output value
    TuningMethod _method; // Current tuning method
    float _setpoint; // Desired setpoint

    // PID parameters
    float _kp, _ki, _kd; // Proportional, integral, and derivative gains
    float _error, _previousError; // Current and previous error values
    float _integral, _derivative; // Integral and derivative terms
    float _output; // Current output value

    // Anti-windup
    bool _antiWindupEnabled; // Flag to enable/disable anti-windup
    float _integralWindupThreshold; // Threshold for integral windup

    // Autotuning parameters
    unsigned long _lastUpdate; // Timestamp of the last update
    float _relayOutput; // Relay output value
    bool _relayState; // Current state of the relay

    // Oscillation detection
    static constexpr int MAX_PEAKS = 10; // Maximum number of peaks to detect
    float _peaks[MAX_PEAKS]; // Array to store detected peaks
    int _peakCount; // Number of peaks detected
    float _lastInput; // Last input value
    bool _risingInput; // Flag to indicate if the input is rising

    // Tuning results
    float _ultimateGain; // Ultimate gain (Ku)
    float _oscillationPeriod; // Oscillation period (Tu)

    // Filtering
    bool _inputFilterEnabled; // Flag to enable/disable input filtering
    bool _outputFilterEnabled; // Flag to enable/disable output filtering
    float _inputFilteredValue; // Filtered input value
    float _outputFilteredValue; // Filtered output value
    float _inputFilterAlpha; // Alpha value for input filtering
    float _outputFilterAlpha; // Alpha value for output filtering

    // Data storage for real-time tuning
    static constexpr int DEFAULT_DATA_POINT_SIZE = 25; // Default size of the data point buffer
    DataPoint _dataPoints[DEFAULT_DATA_POINT_SIZE]; // Array to store data points
    int _dataPointSize; // Current size of the data point buffer
    int _dataIndex; // Index for circular buffer
};

#endif
