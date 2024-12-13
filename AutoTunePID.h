#ifndef AUTOTUNEPID_H
#define AUTOTUNEPID_H

#include <Arduino.h>

enum TuningMethod {
    ZieglerNichols,
    CohenCoon,
    RelayFeedback,
    IMC,
    Manual
};

class AutoTunePID {
public:
    AutoTunePID(float minOutput, float maxOutput, TuningMethod method = ZieglerNichols);

    void setSetpoint(float setpoint);
    void setTuningMethod(TuningMethod method);
    void setManualGains(float kp, float ki, float kd);
    void enableInputFilter(float alpha);
    void enableOutputFilter(float alpha);
    void update(float currentInput);

    float getOutput();
    float getKp();
    float getKi();
    float getKd();

private:
    void computePID();
    void autoTuneZieglerNichols();
    void autoTuneCohenCoon();
    void autoTuneRelayFeedback();
    void autoTuneIMC();
    float applyFilter(float input, float &filteredValue, float alpha);

    float _setpoint;
    float _minOutput;
    float _maxOutput;
    float _kp, _ki, _kd;
    float _error, _previousError;
    float _integral, _derivative;
    float _output;

    float _Ku, _Tu;
    TuningMethod _method;
    unsigned long _lastUpdate;
    unsigned long _tuningStartTime;
    unsigned long _tuningDuration;

    bool _tuning;
    float _maxObservedOutput;
    float _minObservedOutput;

    bool _inputFilterEnabled;
    bool _outputFilterEnabled;
    float _inputFilteredValue;
    float _outputFilteredValue;
    float _inputFilterAlpha;
    float _outputFilterAlpha;
};

#endif
