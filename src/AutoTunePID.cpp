#include "AutoTunePID.h"

namespace atp {

AutoTunePID::AutoTunePID(float minOutput, float maxOutput, TuningMethod method)
    : _minOutput(minOutput)
    , _maxOutput(maxOutput)
    , _method(method)
    , _operationalMode(OperationalMode::Normal)
    , _oscillationMode(OscillationMode::Normal)
    , _oscillationSteps(10)
    , _setpoint(0.0f)
    , _lambda(0.5f)
    , _manualOutput(0.0f)
    , _overrideOutput(0.0f)
    , _trackReference(0.0f)
    , _kp(0.0f)
    , _ki(0.0f)
    , _kd(0.0f)
    , _error(0.0f)
    , _previousError(0.0f)
    , _integral(0.0f)
    , _derivative(0.0f)
    , _output(0.0f)
    , _input(0.0f)
    , _antiWindupEnabled(true)
    , _integralWindupThreshold(0.8f * (maxOutput - minOutput))
    , _lastUpdate(0U)
    , _ultimateGain(0.0f)
    , _oscillationPeriod(0.0f)
    , _processTimeConstant(0.0f)
    , _deadTime(0.0f)
    , _integralTime(0.0f)
    , _derivativeTime(0.0f)
    , _inputFilterEnabled(false)
    , _outputFilterEnabled(false)
    , _inputFilteredValue(0.0f)
    , _outputFilteredValue(0.0f)
    , _inputFilterAlpha(0.1f)
    , _outputFilterAlpha(0.1f)
{
}

void AutoTunePID::setSetpoint(float setpoint)
{
    _setpoint = setpoint;
}

void AutoTunePID::setTuningMethod(TuningMethod method)
{
    _method = method;
}

void AutoTunePID::setManualGains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void AutoTunePID::enableInputFilter(float alpha)
{
    _inputFilterEnabled = true;
    _inputFilterAlpha = constrain(alpha, 0.01f, 1.0f);
}

void AutoTunePID::enableOutputFilter(float alpha)
{
    _outputFilterEnabled = true;
    _outputFilterAlpha = constrain(alpha, 0.01f, 1.0f);
}

void AutoTunePID::enableAntiWindup(bool enable, float threshold)
{
    _antiWindupEnabled = enable;
    _integralWindupThreshold = threshold * (_maxOutput - _minOutput);
}

void AutoTunePID::setOperationalMode(OperationalMode mode)
{
    _operationalMode = mode;
    if (mode == OperationalMode::Hold) {
        _integral = 0.0f;
        _previousError = 0.0f;
        _output = 0.0f;
    }
}

void AutoTunePID::setManualOutput(float output)
{
    _manualOutput = constrain(output, 0.0f, 100.0f);
}

void AutoTunePID::setOverrideOutput(float output)
{
    _overrideOutput = constrain(output, _minOutput, _maxOutput);
}

void AutoTunePID::setTrackReference(float reference)
{
    _trackReference = reference;
}

void AutoTunePID::setOscillationMode(OscillationMode mode)
{
    _oscillationMode = mode;
    switch (mode) {
    case OscillationMode::Normal:
        _oscillationSteps = 10;
        break;
    case OscillationMode::Half:
        _oscillationSteps = 20;
        break;
    case OscillationMode::Mild:
        _oscillationSteps = 40;
        break;
    }
}

void AutoTunePID::setOscillationSteps(int32_t steps)
{
    if (steps > 0) {
        _oscillationSteps = steps;
    }
}

void AutoTunePID::setLambda(float lambda)
{
    _lambda = lambda;
}

void AutoTunePID::update(float currentInput)
{
    const uint32_t now = millis();
    if ((now - _lastUpdate) < 100U) {
        return;
    }

    const float dt = static_cast<float>(now - _lastUpdate) / 1000.0f;
    _lastUpdate = now;

    if (_inputFilterEnabled && (_operationalMode != OperationalMode::Tune)) {
        currentInput = computeFilteredValue(currentInput, _inputFilteredValue, _inputFilterAlpha);
    }
    _input = currentInput;

    if (_operationalMode == OperationalMode::Tune) {
        performAutoTune(currentInput);
    } else if (_operationalMode == OperationalMode::Manual) {
        _output = map(static_cast<long>(_manualOutput), 0L, 100L, static_cast<long>(_minOutput), static_cast<long>(_maxOutput));
        return;
    } else if (_operationalMode == OperationalMode::Override) {
        _output = _overrideOutput;
        return;
    } else if (_operationalMode == OperationalMode::Track) {
        _output = constrain(_trackReference, _minOutput, _maxOutput);
        return;
    } else if (_operationalMode == OperationalMode::Hold) {
        return;
    } else if (_operationalMode == OperationalMode::Preserve) {
        if (_operationalMode == OperationalMode::Reverse) {
            _error = _input - _setpoint;
        } else {
            _error = _setpoint - _input;
        }
        return;
    } else {
        if (_operationalMode == OperationalMode::Reverse) {
            _error = _input - _setpoint;
        } else {
            _error = _setpoint - _input;
        }

        if (abs(_error) < 0.001f) {
            _integral = 0.0f;
        } else {
            _integral += _error * dt;
        }

        _derivative = (_error - _previousError) / dt;
        computePID();
        applyAntiWindup();
        _previousError = _error;
    }

    if (_outputFilterEnabled && (_operationalMode != OperationalMode::Tune)) {
        _output = computeFilteredValue(_output, _outputFilteredValue, _outputFilterAlpha);
    }
}

void AutoTunePID::performAutoTune(float currentInput)
{
    static uint32_t lastToggleTime = 0U;
    static bool outputState = true;
    static int32_t oscillationCount = 0;
    static uint32_t oscillationStartTime = 0U;

    const uint32_t currentTime = millis();

    float highOutput;
    float lowOutput;
    switch (_oscillationMode) {
    case OscillationMode::Normal:
        highOutput = _maxOutput;
        lowOutput = _minOutput;
        break;
    case OscillationMode::Half:
        highOutput = ((_maxOutput + _minOutput) / 2.0f) + ((_maxOutput - _minOutput) / 4.0f);
        lowOutput = ((_maxOutput + _minOutput) / 2.0f) - ((_maxOutput - _minOutput) / 4.0f);
        break;
    case OscillationMode::Mild:
        highOutput = ((_maxOutput + _minOutput) / 2.0f) + ((_maxOutput - _minOutput) / 8.0f);
        lowOutput = ((_maxOutput + _minOutput) / 2.0f) - ((_maxOutput - _minOutput) / 8.0f);
        break;
    default:
        highOutput = _maxOutput;
        lowOutput = _minOutput;
        break;
    }

    if ((currentTime - lastToggleTime) >= 1000U) {
        outputState = !outputState;
        _output = outputState ? highOutput : lowOutput;
        lastToggleTime = currentTime;

        if (oscillationCount == 0) {
            oscillationStartTime = currentTime;
        }
        oscillationCount++;

        if (oscillationCount >= _oscillationSteps) {
            _oscillationPeriod = static_cast<float>(currentTime - oscillationStartTime) / (static_cast<float>(_oscillationSteps) * 1000.0f);

            const float relayAmplitude = (_maxOutput - _minOutput) / 2.0f;
            const float oscillationAmplitude = relayAmplitude;
            _ultimateGain = (4.0f * relayAmplitude) / (kPi * oscillationAmplitude);

            _processTimeConstant = 0.67f * _oscillationPeriod;
            _deadTime = 0.17f * _oscillationPeriod;

            _integralTime = 2.0f * _deadTime;
            _derivativeTime = _deadTime / 2.0f;

            switch (_method) {
            case TuningMethod::ZieglerNichols:
                calculateZieglerNicholsGains();
                break;
            case TuningMethod::CohenCoon:
                calculateCohenCoonGains();
                break;
            case TuningMethod::IMC:
                calculateIMCGains();
                break;
            case TuningMethod::TyreusLuyben:
                calculateTyreusLuybenGains();
                break;
            case TuningMethod::LambdaTuning:
                calculateLambdaTuningGains();
                break;
            default:
                break;
            }

            oscillationCount = 0;
            _operationalMode = OperationalMode::Normal;
        }
    }
}

void AutoTunePID::calculateZieglerNicholsGains()
{
    _kp = 0.6f * _ultimateGain;
    _ki = _kp / _integralTime;
    _kd = _kp * _derivativeTime;
}

void AutoTunePID::calculateCohenCoonGains()
{
    _kp = 0.8f * _ultimateGain;
    _ki = _kp / (0.8f * _oscillationPeriod);
    _kd = 0.194f * _kp * _oscillationPeriod;
}

void AutoTunePID::calculateIMCGains()
{
    float lambda = _lambda;
    if (lambda <= 0.0f) {
        lambda = 0.5f * _processTimeConstant;
    }

    _kp = _processTimeConstant / (lambda + _deadTime);
    _ki = _kp / _processTimeConstant;
    _kd = (_kp * _deadTime) / 2.0f;
}

void AutoTunePID::calculateTyreusLuybenGains()
{
    _kp = 0.45f * _ultimateGain;
    _ki = _kp / _integralTime;
    _kd = 0.0f;
}

void AutoTunePID::calculateLambdaTuningGains()
{
    if (_lambda <= 0.0f) {
        _lambda = 0.5f * _processTimeConstant;
    }

    _kp = _processTimeConstant / (_ultimateGain * (_lambda + _deadTime));
    _ki = _kp / _processTimeConstant;
    _kd = _kp * 0.5f * _deadTime;
}

void AutoTunePID::computePID()
{
    _error = _setpoint - _input;

    if (abs(_error) < 0.001f) {
        _error = 0.0f;
    }

    const float P = _kp * _error;
    const float I = _ki * _integral;
    const float D = _kd * _derivative;

    _output = P + I + D;
    _output = constrain(_output, _minOutput, _maxOutput);
}

void AutoTunePID::applyAntiWindup()
{
    if (_antiWindupEnabled && (abs(_integral) > _integralWindupThreshold)) {
        _integral = constrain(_integral, -_integralWindupThreshold, _integralWindupThreshold);
    }
}

float AutoTunePID::computeFilteredValue(float input, float& filteredValue, float alpha) const
{
    filteredValue = (alpha * input) + ((1.0f - alpha) * filteredValue);
    return filteredValue;
}

} // namespace atp
