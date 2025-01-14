#include "AutoTunePID.h"

AutoTunePID::AutoTunePID(float minOutput, float maxOutput, TuningMethod method)
    : _minOutput(minOutput)
    , _maxOutput(maxOutput)
    , _method(method)
    , _kp(0)
    , _ki(0)
    , _kd(0)
    , _error(0)
    , _previousError(0)
    , _integral(0)
    , _output(0)
    , _tuningState(TuningState::Initial)
    , _lastUpdate(0)
    , _tuningStartTime(0)
    , _tuningDuration(30000)
    , _relayOutput(maxOutput)
    , _relayState(true)
    , _peakCount(0)
    , _lastInput(0)
    , _risingInput(true)
    , _ultimateGain(0)
    , _oscillationPeriod(0)
    , _inputFilterEnabled(false)
    , _outputFilterEnabled(false)
    , _inputFilteredValue(0)
    , _outputFilteredValue(0)
    , _inputFilterAlpha(0.1)
    , _outputFilterAlpha(0.1)
{
    for (int i = 0; i < MAX_PEAKS; i++) {
        _peaks[i] = 0;
    }
}

void AutoTunePID::setSetpoint(float setpoint)
{
    _setpoint = setpoint;
    if (_tuningState != TuningState::Complete) {
        initializeAutoTune();
    }
}

void AutoTunePID::setTuningMethod(TuningMethod method)
{
    _method = method;
    if (method != TuningMethod::Manual) {
        initializeAutoTune();
    }
}

void AutoTunePID::setManualGains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _tuningState = TuningState::Complete;
}

void AutoTunePID::enableInputFilter(float alpha)
{
    _inputFilterEnabled = true;
    _inputFilterAlpha = constrain(alpha, 0.01, 1.0);
}

void AutoTunePID::enableOutputFilter(float alpha)
{
    _outputFilterEnabled = true;
    _outputFilterAlpha = constrain(alpha, 0.01, 1.0);
}

void AutoTunePID::update(float currentInput)
{
    unsigned long now = millis();
    if (now - _lastUpdate < 100)
        return; // Maintain consistent sample time

    _lastUpdate = now;

    // Apply input filtering if enabled
    if (_inputFilterEnabled) {
        currentInput = computeFilteredValue(currentInput, _inputFilteredValue, _inputFilterAlpha);
    }

    if (_tuningState != TuningState::Complete && _method != TuningMethod::Manual) {
        performAutoTune(currentInput);
    } else {
        // Normal PID control
        _error = _setpoint - currentInput;
        _integral = constrain(_integral + _error, _minOutput, _maxOutput);
        _derivative = _error - _previousError;

        computePID();
        _previousError = _error;
    }

    // Apply output filtering if enabled
    if (_outputFilterEnabled) {
        _output = computeFilteredValue(_output, _outputFilteredValue, _outputFilterAlpha);
    }
}

void AutoTunePID::performAutoTune(float currentInput)
{
    if (_tuningState == TuningState::Initial) {
        _tuningStartTime = millis();
        _tuningState = TuningState::Oscillating;
        _relayState = currentInput < _setpoint;
    }

    processAutoTune(currentInput);

    // Check if tuning duration has elapsed
    if (millis() - _tuningStartTime >= _tuningDuration) {
        finalizeTuning();
    }
}

void AutoTunePID::processAutoTune(float currentInput)
{
    // Relay feedback method
    _relayState = currentInput < _setpoint;
    _output = _relayState ? _maxOutput : _minOutput;

    // Peak detection
    bool risingInput = currentInput > _lastInput;
    if (_risingInput != risingInput && _peakCount < MAX_PEAKS) {
        _peaks[_peakCount++] = _lastInput;
        _risingInput = risingInput;
    }
    _lastInput = currentInput;
}

void AutoTunePID::finalizeTuning()
{
    if (_peakCount < 2)
        return;

    // Calculate average peak-to-peak amplitude and period
    float amplitude = 0;
    for (int i = 1; i < _peakCount; i += 2) {
        amplitude += abs(_peaks[i] - _peaks[i - 1]);
    }
    amplitude /= (_peakCount / 2);

    _oscillationPeriod = (_tuningDuration / 1000.0f) / (_peakCount / 2);
    _ultimateGain = (4.0f * (_maxOutput - _minOutput)) / (PI * amplitude);

    switch (_method) {
    case TuningMethod::ZieglerNichols:
        calculateZieglerNicholsGains();
        break;
    case TuningMethod::CohenCoon:
        calculateCohenCoonGains();
        break;
    case TuningMethod::RelayFeedback:
        calculateRelayFeedbackGains();
        break;
    case TuningMethod::IMC:
        calculateIMCGains();
        break;
    default:
        break;
    }

    _tuningState = TuningState::Complete;
}

void AutoTunePID::calculateZieglerNicholsGains()
{
    _kp = 0.6f * _ultimateGain;
    _ki = 1.2f * _kp / _oscillationPeriod;
    _kd = 0.075f * _kp * _oscillationPeriod;
}

void AutoTunePID::calculateCohenCoonGains()
{
    _kp = 0.8f * _ultimateGain;
    _ki = _kp / (0.8f * _oscillationPeriod);
    _kd = 0.194f * _kp * _oscillationPeriod;
}

void AutoTunePID::calculateRelayFeedbackGains()
{
    _kp = 0.5f * _ultimateGain;
    _ki = 1.0f * _kp / _oscillationPeriod;
    _kd = 0.125f * _kp * _oscillationPeriod;
}

void AutoTunePID::calculateIMCGains()
{
    const float lambda = 0.5f * _oscillationPeriod;
    _kp = 0.4f * _ultimateGain;
    _ki = _kp / (2.0f * lambda);
    _kd = 0.5f * _kp * lambda;
}

void AutoTunePID::computePID()
{
    _output = (_kp * _error) + (_ki * _integral) + (_kd * _derivative);
    _output = constrain(_output, _minOutput, _maxOutput);
}

float AutoTunePID::computeFilteredValue(float input, float& filteredValue, float alpha) const
{
    filteredValue = (alpha * input) + ((1.0f - alpha) * filteredValue);
    return filteredValue;
}

void AutoTunePID::initializeAutoTune()
{
    _tuningState = TuningState::Initial;
    _peakCount = 0;
    _lastInput = 0;
    _risingInput = true;
    _ultimateGain = 0;
    _oscillationPeriod = 0;
}
