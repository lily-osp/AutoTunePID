#include "AutoTunePID.h"

AutoTunePID::AutoTunePID(float minOutput, float maxOutput, TuningMethod method)
    : _minOutput(minOutput)
    , _maxOutput(maxOutput)
    , _method(method)
    , _operationalMode(OperationalMode::Normal)
    , _oscillationMode(OscillationMode::Normal)
    , _oscillationSteps(10)
    , _setpoint(0)
    , _lambda(0.5f)
    , _kp(0)
    , _ki(0)
    , _kd(0)
    , _error(0)
    , _previousError(0)
    , _integral(0)
    , _output(0)
    , _lastUpdate(0)
    , _ultimateGain(0)
    , _oscillationPeriod(0)
    , _processTimeConstant(0)
    , _deadTime(0)
    , _integralTime(0)
    , _derivativeTime(0)
    , _inputFilterEnabled(false)
    , _outputFilterEnabled(false)
    , _inputFilteredValue(0)
    , _outputFilteredValue(0)
    , _inputFilterAlpha(0.1)
    , _outputFilterAlpha(0.1)
    , _antiWindupEnabled(true)
    , _integralWindupThreshold(0.8f * (maxOutput - minOutput))
    , _correctorEnabled(false)
    , _dataWindowSize(10)
    , _stabilityThreshold(0.1f)
    , _dataPoints(nullptr)
    , _dataIndex(0)
{
    _dataPoints = new float[_dataWindowSize];
}

AutoTunePID::~AutoTunePID()
{
    if (_dataPoints) {
        delete[] _dataPoints;
    }
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
    _inputFilterAlpha = constrain(alpha, 0.01, 1.0);
}

void AutoTunePID::enableOutputFilter(float alpha)
{
    _outputFilterEnabled = true;
    _outputFilterAlpha = constrain(alpha, 0.01, 1.0);
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
        _integral = 0;
        _previousError = 0;
        _output = 0;
    }
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

void AutoTunePID::setOscillationSteps(int steps)
{
    if (steps > 0) {
        _oscillationSteps = steps;
    }
}

void AutoTunePID::setLambda(float lambda)
{
    _lambda = lambda;
}

void AutoTunePID::enableCorrector(bool enable, int dataWindowSize, float stabilityThreshold)
{
    _correctorEnabled = enable;
    _dataWindowSize = dataWindowSize;
    _stabilityThreshold = stabilityThreshold;

    if (_dataPoints) {
        delete[] _dataPoints;
    }
    _dataPoints = new float[_dataWindowSize];
    _dataIndex = 0;
}

bool AutoTunePID::isSystemUnstable(const float* dataPoints, int dataSize)
{
    if (!_correctorEnabled || dataSize < 2) {
        return false;
    }

    float totalDifference = 0;
    for (int i = 1; i < dataSize; i++) {
        totalDifference += abs(dataPoints[i] - dataPoints[i - 1]);
    }
    float averageDifference = totalDifference / (dataSize - 1);

    return averageDifference > _stabilityThreshold;
}

void AutoTunePID::applyCorrector(const float* dataPoints, int dataSize)
{
    if (isSystemUnstable(dataPoints, dataSize)) {
        _integral = 0;
        _output = constrain(_output * 0.5f, _minOutput, _maxOutput);

        if (dataSize >= _dataWindowSize) {
            setOperationalMode(OperationalMode::Tune);
        }
    }
}

void AutoTunePID::update(float currentInput)
{
    unsigned long now = millis();
    if (now - _lastUpdate < 100)
        return;
    _lastUpdate = now;

    if (_inputFilterEnabled && _operationalMode != OperationalMode::Tune) {
        currentInput = computeFilteredValue(currentInput, _inputFilteredValue, _inputFilterAlpha);
    }
    _input = currentInput;

    if (_correctorEnabled) {
        _dataPoints[_dataIndex] = currentInput;
        _dataIndex = (_dataIndex + 1) % _dataWindowSize;
    }

    if (_operationalMode == OperationalMode::Tune) {
        performAutoTune(currentInput);
    } else {
        if (_correctorEnabled) {
            applyCorrector(_dataPoints, _dataWindowSize);
        }

        _error = _setpoint - _input;

        if (abs(_error) < 0.001) {
            _integral = 0;
        } else {
            _integral += _error * 0.1f;
        }

        _derivative = _error - _previousError;
        computePID();
        applyAntiWindup();
        _previousError = _error;
    }

    if (_outputFilterEnabled && _operationalMode != OperationalMode::Tune) {
        _output = computeFilteredValue(_output, _outputFilteredValue, _outputFilterAlpha);
    }
}

void AutoTunePID::performAutoTune(float currentInput)
{
    static unsigned long lastToggleTime = 0;
    static bool outputState = true;
    static int oscillationCount = 0;
    static unsigned long oscillationStartTime = 0;

    unsigned long currentTime = millis();

    float highOutput, lowOutput;
    switch (_oscillationMode) {
    case OscillationMode::Normal:
        highOutput = _maxOutput;
        lowOutput = _minOutput;
        break;
    case OscillationMode::Half:
        highOutput = (_maxOutput + _minOutput) / 2.0f + (_maxOutput - _minOutput) / 4.0f;
        lowOutput = (_maxOutput + _minOutput) / 2.0f - (_maxOutput - _minOutput) / 4.0f;
        break;
    case OscillationMode::Mild:
        highOutput = (_maxOutput + _minOutput) / 2.0f + (_maxOutput - _minOutput) / 8.0f;
        lowOutput = (_maxOutput + _minOutput) / 2.0f - (_maxOutput - _minOutput) / 8.0f;
        break;
    }

    if (currentTime - lastToggleTime >= 1000) {
        outputState = !outputState;
        _output = outputState ? highOutput : lowOutput;
        lastToggleTime = currentTime;

        if (oscillationCount == 0) {
            oscillationStartTime = currentTime;
        }
        oscillationCount++;

        if (oscillationCount >= _oscillationSteps) {
            _oscillationPeriod = (currentTime - oscillationStartTime) / (float)(_oscillationSteps * 1000);
            _ultimateGain = (4.0f * (highOutput - lowOutput)) / (PI * (highOutput - lowOutput));

            _processTimeConstant = _oscillationPeriod / 2.0f;
            _deadTime = _oscillationPeriod / 4.0f;

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
    _kp = (1.35f / _ultimateGain) * (_processTimeConstant / _deadTime + 0.185f);
    _ki = _kp / (_processTimeConstant + 0.611f * _deadTime);
    _kd = _kp * 0.185f * _deadTime;
}

void AutoTunePID::calculateIMCGains()
{
    const float lambda = 0.5f * _oscillationPeriod;
    _kp = _processTimeConstant / (lambda + _deadTime);
    _ki = _kp / _integralTime;
    _kd = _kp * _derivativeTime;
}

void AutoTunePID::calculateTyreusLuybenGains()
{
    _kp = 0.45f * _ultimateGain;
    _ki = _kp / _integralTime;
    _kd = 0.0f;
}

void AutoTunePID::calculateLambdaTuningGains()
{
    if (_lambda <= 0) {
        _lambda = 0.5f * _processTimeConstant;
    }
    _kp = _processTimeConstant / (_ultimateGain * (_lambda + _deadTime));
    _ki = _kp / _processTimeConstant;
    _kd = _kp * 0.5f * _deadTime;
}

void AutoTunePID::computePID()
{
    _error = _setpoint - _input;

    if (abs(_error) < 0.001) {
        _error = 0;
    }

    float P = _kp * _error;
    float I = _ki * _integral;
    float D = _kd * _derivative;

    _output = P + I + D;
    _output = constrain(_output, _minOutput, _maxOutput);
}

void AutoTunePID::applyAntiWindup()
{
    if (_antiWindupEnabled && abs(_integral) > _integralWindupThreshold) {
        _integral = constrain(_integral, -_integralWindupThreshold, _integralWindupThreshold);
    }
}

float AutoTunePID::computeFilteredValue(float input, float& filteredValue, float alpha) const
{
    filteredValue = (alpha * input) + ((1.0f - alpha) * filteredValue);
    return filteredValue;
}
