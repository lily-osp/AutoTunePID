#ifndef AUTOTUNEPID_H
#define AUTOTUNEPID_H

#include <Arduino.h>

enum class TuningMethod {
    ZieglerNichols,
    CohenCoon,
    RelayFeedback,
    IMC,
    Manual
};

// Backward compatibility
constexpr auto ZieglerNichols = TuningMethod::ZieglerNichols;
constexpr auto CohenCoon = TuningMethod::CohenCoon;
constexpr auto RelayFeedback = TuningMethod::RelayFeedback;
constexpr auto IMC = TuningMethod::IMC;
constexpr auto Manual = TuningMethod::Manual;

enum class TuningState {
    Initial,
    Oscillating,
    Complete
};

class AutoTunePID {
public:
    AutoTunePID(float minOutput, float maxOutput, TuningMethod method = TuningMethod::ZieglerNichols);

    // Configuration methods
    void setSetpoint(float setpoint);
    void setTuningMethod(TuningMethod method);
    void setManualGains(float kp, float ki, float kd);
    void enableInputFilter(float alpha);
    void enableOutputFilter(float alpha);
    void setTuningDuration(unsigned long durationMs) { _tuningDuration = durationMs; }

    // Runtime methods
    void update(float currentInput);
    float getOutput() const { return _output; }
    float getKp() const { return _kp; }
    float getKi() const { return _ki; }
    float getKd() const { return _kd; }
    bool isTuning() const { return _tuningState != TuningState::Complete; }

private:
    // PID computation
    void computePID();
    float computeFilteredValue(float input, float& filteredValue, float alpha) const;

    // Autotuning methods
    void performAutoTune(float currentInput);
    void initializeAutoTune();
    void processAutoTune(float currentInput);
    void finalizeTuning();
    void calculateZieglerNicholsGains();
    void calculateCohenCoonGains();
    void calculateRelayFeedbackGains();
    void calculateIMCGains();

    // Configuration
    const float _minOutput;
    const float _maxOutput;
    TuningMethod _method;
    float _setpoint;

    // PID parameters
    float _kp, _ki, _kd;
    float _error, _previousError;
    float _integral, _derivative;
    float _output;

    // Autotuning parameters
    TuningState _tuningState;
    unsigned long _lastUpdate;
    unsigned long _tuningStartTime;
    unsigned long _tuningDuration;
    float _relayOutput;
    bool _relayState;

    // Oscillation detection
    static constexpr int MAX_PEAKS = 10;
    float _peaks[MAX_PEAKS];
    int _peakCount;
    float _lastInput;
    bool _risingInput;

    // Tuning results
    float _ultimateGain; // Ku
    float _oscillationPeriod; // Tu

    // Filtering
    bool _inputFilterEnabled;
    bool _outputFilterEnabled;
    float _inputFilteredValue;
    float _outputFilteredValue;
    float _inputFilterAlpha;
    float _outputFilterAlpha;
};

#endif
