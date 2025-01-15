#include "AutoTunePID.h"

// Constructor implementation
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
    , _lastUpdate(0)
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
    , _dataPointSize(DEFAULT_DATA_POINT_SIZE)
    , _dataIndex(0)
    , _antiWindupEnabled(true)
    , _integralWindupThreshold(0.8f * (maxOutput - minOutput))
{
    // Initialize peaks array
    for (int i = 0; i < MAX_PEAKS; i++) {
        _peaks[i] = 0;
    }
}

// Set the size of the data point buffer
void AutoTunePID::setDataPointSize(int size)
{
    if (size > 0 && size <= DEFAULT_DATA_POINT_SIZE) {
        _dataPointSize = size;
        _dataIndex = 0; // Reset index when changing size
    }
}

// Set the desired setpoint
void AutoTunePID::setSetpoint(float setpoint)
{
    _setpoint = setpoint;
}

// Set the tuning method
void AutoTunePID::setTuningMethod(TuningMethod method)
{
    _method = method;
}

// Set manual PID gains
void AutoTunePID::setManualGains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

// Enable input filtering with a given alpha value
void AutoTunePID::enableInputFilter(float alpha)
{
    _inputFilterEnabled = true;
    _inputFilterAlpha = constrain(alpha, 0.01, 1.0);
}

// Enable output filtering with a given alpha value
void AutoTunePID::enableOutputFilter(float alpha)
{
    _outputFilterEnabled = true;
    _outputFilterAlpha = constrain(alpha, 0.01, 1.0);
}

// Enable/disable anti-windup with optional threshold
void AutoTunePID::enableAntiWindup(bool enable, float threshold)
{
    _antiWindupEnabled = enable;
    _integralWindupThreshold = threshold * (_maxOutput - _minOutput);
}

// Update the PID controller with the current input
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

    // Store the latest data point in the circular buffer
    _dataPoints[_dataIndex] = { currentInput, _setpoint, _output, now };
    _dataIndex = (_dataIndex + 1) % _dataPointSize; // Circular buffer logic

    // Perform real-time tuning and PID control
    performAutoTune(currentInput);

    // Normal PID control
    _error = _setpoint - currentInput;
    _integral += _error;
    _derivative = _error - _previousError;

    computePID();
    applyAntiWindup(); // Apply anti-windup
    _previousError = _error;

    // Apply output filtering if enabled
    if (_outputFilterEnabled) {
        _output = computeFilteredValue(_output, _outputFilteredValue, _outputFilterAlpha);
    }
}

// Apply anti-windup to prevent integral windup
void AutoTunePID::applyAntiWindup()
{
    if (_antiWindupEnabled && abs(_integral) > _integralWindupThreshold) {
        _integral = constrain(_integral, -_integralWindupThreshold, _integralWindupThreshold);
    }
}

// Perform auto-tuning based on the current input
void AutoTunePID::performAutoTune(float currentInput)
{
    if (_dataIndex < _dataPointSize)
        return; // Wait until the buffer is filled

    // Relay feedback logic
    if (currentInput < _setpoint) {
        _output = _maxOutput; // Turn on relay
    } else {
        _output = _minOutput; // Turn off relay
    }

    // Peak detection logic
    bool risingInput = currentInput > _lastInput;
    if (_risingInput != risingInput) {
        if (_peakCount < MAX_PEAKS) {
            _peaks[_peakCount++] = _lastInput; // Store the peak value
        }
        _risingInput = risingInput;
    }
    _lastInput = currentInput;

    // Calculate Ku and Tu if enough peaks are detected
    if (_peakCount >= 2) {
        float amplitude = 0;
        for (int i = 1; i < _peakCount; i++) {
            amplitude += abs(_peaks[i] - _peaks[i - 1]);
        }
        amplitude /= (_peakCount - 1);

        unsigned long periodSum = 0;
        for (int i = 1; i < _peakCount; i++) {
            periodSum += _dataPoints[(_dataIndex + i) % _dataPointSize].timestamp - _dataPoints[(_dataIndex + i - 1) % _dataPointSize].timestamp;
        }
        _oscillationPeriod = periodSum / (float)(_peakCount - 1) / 1000.0f; // Convert to seconds

        _ultimateGain = (4.0f * (_maxOutput - _minOutput)) / (PI * amplitude);

        // Calculate PID gains based on the selected tuning method
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
        case TuningMethod::TyreusLuyben:
            calculateTyreusLuybenGains();
            break;
        default:
            break;
        }

        // Reset peak count after calculating gains
        _peakCount = 0;
    }
}

// Get the amplitude of oscillations
float AutoTunePID::getAmplitude() const
{
    if (_peakCount < 2)
        return 0.0f; // Not enough peaks to calculate amplitude
    float amplitude = 0;
    for (int i = 1; i < _peakCount; i++) {
        amplitude += abs(_peaks[i] - _peaks[i - 1]);
    }
    return amplitude / (_peakCount - 1);
}

// Get the peak value at the specified index
float AutoTunePID::getPeak(int index) const
{
    if (index < 0 || index >= _peakCount)
        return 0.0f; // Invalid index
    return _peaks[index];
}

// Calculate PID gains using Ziegler-Nichols method
void AutoTunePID::calculateZieglerNicholsGains()
{
    _kp = 0.6f * _ultimateGain;
    _ki = 1.2f * _kp / _oscillationPeriod;
    _kd = 0.075f * _kp * _oscillationPeriod;
}

// Calculate PID gains using Cohen-Coon method
void AutoTunePID::calculateCohenCoonGains()
{
    _kp = 0.8f * _ultimateGain;
    _ki = _kp / (0.8f * _oscillationPeriod);
    _kd = 0.194f * _kp * _oscillationPeriod;
}

// Calculate PID gains using Relay Feedback method
void AutoTunePID::calculateRelayFeedbackGains()
{
    _kp = 0.5f * _ultimateGain;
    _ki = 1.0f * _kp / _oscillationPeriod;
    _kd = 0.125f * _kp * _oscillationPeriod;
}

// Calculate PID gains using IMC method
void AutoTunePID::calculateIMCGains()
{
    const float lambda = 0.5f * _oscillationPeriod;
    _kp = 0.4f * _ultimateGain;
    _ki = _kp / (2.0f * lambda);
    _kd = 0.5f * _kp * lambda;
}

// Calculate PID gains using Tyreus-Luyben method
void AutoTunePID::calculateTyreusLuybenGains()
{
    _kp = 0.45f * _ultimateGain;
    _ki = _kp / (2.2f * _oscillationPeriod);
    _kd = 0.0f; // Tyreus-Luyben typically does not use derivative gain
}

// Compute the PID output
void AutoTunePID::computePID()
{
    _output = (_kp * _error) + (_ki * _integral) + (_kd * _derivative);
    _output = constrain(_output, _minOutput, _maxOutput);
}

// Compute filtered value using exponential moving average
float AutoTunePID::computeFilteredValue(float input, float& filteredValue, float alpha) const
{
    filteredValue = (alpha * input) + ((1.0f - alpha) * filteredValue);
    return filteredValue;
}
