#include "AutoTunePID.h"

AutoTunePID::AutoTunePID(float minOutput, float maxOutput, TuningMethod method)
    : _minOutput(minOutput)
    , _maxOutput(maxOutput)
    , _method(method)
    , _operationalMode(OperationalMode::Normal)
    , _oscillationMode(OscillationMode::Normal)
    , _oscillationSteps(10)
    , _setpoint(0)
    , _lambda(0.5f) // Default lambda value
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
    , _processTimeConstant(0) // Initialize process time constant (T)
    , _deadTime(0) // Initialize dead time (L)
    , _integralTime(0) // Initialize integral time (Ti)
    , _derivativeTime(0) // Initialize derivative time (Td)
    , _inputFilterEnabled(false)
    , _outputFilterEnabled(false)
    , _inputFilteredValue(0)
    , _outputFilteredValue(0)
    , _inputFilterAlpha(0.1)
    , _outputFilterAlpha(0.1)
    , _antiWindupEnabled(true)
    , _integralWindupThreshold(0.8f * (maxOutput - minOutput))
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
    // Set default oscillation steps based on the oscillation mode
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

void AutoTunePID::update(float currentInput)
{
    unsigned long now = millis();
    if (now - _lastUpdate < 100)
        return; // Maintain consistent sample time
    _lastUpdate = now;

    // Update input (with filter if enabled)
    if (_inputFilterEnabled && _operationalMode != OperationalMode::Tune) {
        currentInput = computeFilteredValue(currentInput, _inputFilteredValue, _inputFilterAlpha);
    }
    _input = currentInput; // Store the current input value

    if (_operationalMode == OperationalMode::Tune) {
        performAutoTune(currentInput);
    } else {
        _error = _setpoint - _input;

        // Reset integral term if error is zero (faster and smoother zeroing)
        if (abs(_error) < 0.001) {
            _integral = 0;
        } else {
            _integral += _error * 0.1f; // Smoother integral accumulation
        }

        _derivative = _error - _previousError;
        computePID();
        applyAntiWindup();
        _previousError = _error;
    }

    // Update output (with filter if enabled)
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

    // Determine the output range based on the oscillation mode
    float highOutput, lowOutput;
    switch (_oscillationMode) {
    case OscillationMode::Normal:
        highOutput = _maxOutput;
        lowOutput = _minOutput;
        break;
    case OscillationMode::Half:
        highOutput = (_maxOutput + _minOutput) / 2.0f + (_maxOutput - _minOutput) / 4.0f; // 3/4 of the range
        lowOutput = (_maxOutput + _minOutput) / 2.0f - (_maxOutput - _minOutput) / 4.0f; // 1/4 of the range
        break;
    case OscillationMode::Mild:
        highOutput = (_maxOutput + _minOutput) / 2.0f + (_maxOutput - _minOutput) / 8.0f; // 5/8 of the range
        lowOutput = (_maxOutput + _minOutput) / 2.0f - (_maxOutput - _minOutput) / 8.0f; // 3/8 of the range
        break;
    }

    // Toggle output every second to induce oscillations
    if (currentTime - lastToggleTime >= 1000) {
        outputState = !outputState;
        _output = outputState ? highOutput : lowOutput;
        lastToggleTime = currentTime;

        if (oscillationCount == 0) {
            oscillationStartTime = currentTime;
        }
        oscillationCount++;

        // After the specified number of oscillations, calculate Ku and Tu
        if (oscillationCount >= _oscillationSteps) {
            _oscillationPeriod = (currentTime - oscillationStartTime) / (float)(_oscillationSteps * 1000); // Period in seconds
            _ultimateGain = (4.0f * (highOutput - lowOutput)) / (PI * (highOutput - lowOutput)); // Simplified amplitude

            // Estimate T and L from the system response
            _processTimeConstant = _oscillationPeriod / 2.0f; // Approximate T as half the oscillation period
            _deadTime = _oscillationPeriod / 4.0f; // Approximate L as a quarter of the oscillation period

            // Calculate Ti and Td based on T and L
            _integralTime = 2.0f * _deadTime;
            _derivativeTime = _deadTime / 2.0f;

            // Calculate PID gains based on the selected tuning method
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

            // Reset oscillation count and return to normal operation
            oscillationCount = 0;
            _operationalMode = OperationalMode::Normal; // Switch back to Normal mode after tuning
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
    // Ensure lambda is set and valid
    if (_lambda <= 0) {
        _lambda = 0.5f * _processTimeConstant; // Default value for lambda
    }

    // Calculate Kp, Ki, and Kd using the Lambda Tuning (CLD) formula
    _kp = _processTimeConstant / (_ultimateGain * (_lambda + _deadTime));
    _ki = _kp / _processTimeConstant;
    _kd = _kp * 0.5f * _deadTime;
}

void AutoTunePID::computePID()
{
    // Calculate error
    _error = _setpoint - _input;

    // If error is very small, treat it as zero
    if (abs(_error) < 0.001) {
        _error = 0;
    }

    // Calculate PID terms
    float P = _kp * _error;
    float I = _ki * _integral;
    float D = _kd * _derivative;

    // Calculate output
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
