#include "AutoTunePID.h"

AutoTunePID::AutoTunePID(float minOutput, float maxOutput, TuningMethod method)
    : _minOutput(minOutput), _maxOutput(maxOutput), _method(method), _kp(0), _ki(0), _kd(0),
      _error(0), _previousError(0), _integral(0), _output(0), _tuning(true), _lastUpdate(0),
      _tuningStartTime(0), _tuningDuration(5000), _maxObservedOutput(0), _minObservedOutput(0),
      _inputFilterEnabled(false), _outputFilterEnabled(false), _inputFilteredValue(0),
      _outputFilteredValue(0), _inputFilterAlpha(0.1), _outputFilterAlpha(0.1) {}

void AutoTunePID::setSetpoint(float setpoint) {
    _setpoint = setpoint;
}

void AutoTunePID::setTuningMethod(TuningMethod method) {
    _method = method;
}

void AutoTunePID::setManualGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _tuning = false;
}

void AutoTunePID::enableInputFilter(float alpha) {
    _inputFilterEnabled = true;
    _inputFilterAlpha = constrain(alpha, 0.01, 1.0);
}

void AutoTunePID::enableOutputFilter(float alpha) {
    _outputFilterEnabled = true;
    _outputFilterAlpha = constrain(alpha, 0.01, 1.0);
}

void AutoTunePID::update(float currentInput) {
    unsigned long now = millis();
    if (now - _lastUpdate >= 100) {
        _lastUpdate = now;

        if (_inputFilterEnabled) {
            currentInput = applyFilter(currentInput, _inputFilteredValue, _inputFilterAlpha);
        }

        _error = _setpoint - currentInput;
        _integral += _error;
        _derivative = _error - _previousError;

        if (_tuning) {
            if (_method == ZieglerNichols) {
                autoTuneZieglerNichols();
            } else if (_method == CohenCoon) {
                autoTuneCohenCoon();
            } else if (_method == RelayFeedback) {
                autoTuneRelayFeedback();
            } else if (_method == IMC) {
                autoTuneIMC();
            }
        } else {
            computePID();
        }

        if (_outputFilterEnabled) {
            _output = applyFilter(_output, _outputFilteredValue, _outputFilterAlpha);
        }

        _previousError = _error;
    }
}

float AutoTunePID::getOutput() {
    return _output;
}

float AutoTunePID::getKp() {
    return _kp;
}

float AutoTunePID::getKi() {
    return _ki;
}

float AutoTunePID::getKd() {
    return _kd;
}

void AutoTunePID::computePID() {
    _output = (_kp * _error) + (_ki * _integral) + (_kd * _derivative);
    _output = constrain(_output, _minOutput, _maxOutput);
}

void AutoTunePID::autoTuneZieglerNichols() {
    _maxObservedOutput = max(_maxObservedOutput, _output);
    _minObservedOutput = min(_minObservedOutput, _output);

    if (millis() - _tuningStartTime > _tuningDuration) {
        _tuning = false;
        _Ku = 4 * (_maxObservedOutput - _minObservedOutput) / (PI * (_maxObservedOutput + _minObservedOutput));
        _Tu = _tuningDuration / 1000.0;

        _kp = 0.6 * _Ku;
        _ki = 2 * _kp / _Tu;
        _kd = _kp * _Tu / 8;
    }
}

void AutoTunePID::autoTuneCohenCoon() {
    _maxObservedOutput = max(_maxObservedOutput, _output);
    _minObservedOutput = min(_minObservedOutput, _output);

    if (millis() - _tuningStartTime > _tuningDuration) {
        _tuning = false;
        _Ku = 4 * (_maxObservedOutput - _minObservedOutput) / (PI * (_maxObservedOutput + _minObservedOutput));
        _Tu = _tuningDuration / 1000.0;

        _kp = 1.35 * _Ku;
        _ki = _kp / (2.5 * _Tu);
        _kd = 0.37 * _kp * _Tu;
    }
}

void AutoTunePID::autoTuneRelayFeedback() {
    _maxObservedOutput = max(_maxObservedOutput, _output);
    _minObservedOutput = min(_minObservedOutput, _output);

    if (millis() - _tuningStartTime > _tuningDuration) {
        _tuning = false;
        _Ku = 4 * (_maxObservedOutput - _minObservedOutput) / (PI * (_maxObservedOutput + _minObservedOutput));
        _Tu = _tuningDuration / 1000.0;

        _kp = 0.6 * _Ku;
        _ki = 1.2 * _kp / _Tu;
        _kd = 0.075 * _kp * _Tu;
    }
}

void AutoTunePID::autoTuneIMC() {
    const float lambda = 0.5;
    if (millis() - _tuningStartTime > _tuningDuration) {
        _tuning = false;
        _Ku = (_maxOutput - _minOutput) / (_maxObservedOutput + _minObservedOutput);
        _Tu = _tuningDuration / 1000.0;

        _kp = _Ku / (lambda + _Tu);
        _ki = _kp / (_Tu + lambda);
        _kd = _kp * (_Tu * lambda) / (_Tu + lambda);
    }
}

float AutoTunePID::applyFilter(float input, float &filteredValue, float alpha) {
    filteredValue = (alpha * input) + ((1 - alpha) * filteredValue);
    return filteredValue;
}
