/**
 * @file AutoTunePID.cpp
 * @brief Implementation of the AutoTunePID library.
 * @details Adheres to AUTOSAR C++14 standards for type safety and deterministic behavior.
 */

#include "AutoTunePID.h"

namespace atp {

/**
 * @brief Constructor for AutoTunePID.
 * @details Initializes all member variables in the exact order of declaration to satisfy Rule A12-1-6.
 */
AutoTunePID::AutoTunePID(float minOutput, float maxOutput, TuningMethod method)
    : _minOutput(minOutput)
    , _maxOutput(maxOutput)
    , _method(method)
    , _operationalMode(OperationalMode::Normal)
    , _oscillationMode(OscillationMode::Normal)
    , _oscillationSteps(10)
    , _setpoint(0.0f)
    , _lambda(0.5f)
    , _reverseMode(false)
    , _manualOutput(0.0f)
    , _overrideOutput(0.0f)
    , _trackReference(0.0f)
    , _kp(0.0f)
    , _ki(0.0f)
    , _kd(0.0f)
    , _error(0.0f)
    , _previousInput(0.0f)
    , _integral(0.0f)
    , _derivative(0.0f)
    , _output(0.0f)
    , _input(0.0f)
    , _antiWindupEnabled(true)
    , _integralWindupThreshold(0.8f * fabsf(maxOutput - minOutput))
    , _lastUpdate(0U)
    , _ultimateGain(0.0f)
    , _oscillationPeriod(0.0f)
    , _maxInput(-FLT_MAX)
    , _minInput(FLT_MAX)
    , _lastPeakTime(0U)
    , _tuneInProgress(false)
    , _outputState(true)
    , _halfCycleCount(0)
    , _lastCrossingTime(0U)
    , _peakAmplitudeSum(0.0f)
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

/** @brief Sets the controller setpoint. */
void AutoTunePID::setSetpoint(float setpoint)
{
    if (isfinite(setpoint)) {
        _setpoint = setpoint;
    }
}

/** @brief Sets the auto-tuning method. */
void AutoTunePID::setTuningMethod(TuningMethod method)
{
    _method = method;
}

/** @brief Manually sets PID gains. */
void AutoTunePID::setManualGains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

/** @brief Configures input filtering. */
void AutoTunePID::enableInputFilter(float alpha)
{
    _inputFilterEnabled = true;
    _inputFilterAlpha = constrain(alpha, 0.01f, 1.0f);
}

/** @brief Configures output filtering. */
void AutoTunePID::enableOutputFilter(float alpha)
{
    _outputFilterEnabled = true;
    _outputFilterAlpha = constrain(alpha, 0.01f, 1.0f);
}

/** @brief Configures anti-windup protection. */
void AutoTunePID::enableAntiWindup(bool enable, float threshold)
{
    _antiWindupEnabled = enable;
    // Use fabsf to prevent sign reversal when maxOutput < minOutput
    _integralWindupThreshold = threshold * fabsf(_maxOutput - _minOutput);
}

/** @brief Sets operational mode and resets state on 'Hold'. */
void AutoTunePID::setOperationalMode(OperationalMode mode)
{
    _operationalMode = mode;
    
    // Track reverse mode for error direction branching
    if (mode == OperationalMode::Reverse) {
        _reverseMode = true;
    } else if (mode == OperationalMode::Normal) {
        _reverseMode = false;
    }

    if (mode == OperationalMode::Hold) {
        _integral = 0.0f;
        _previousInput = 0.0f;
        _output = 0.0f;
    }
    else if (mode == OperationalMode::Tune) {
        // Reset state variables when re-entering TUNE mode
        _tuneInProgress = true;
        _outputState = true;
        _halfCycleCount = 0;
        _lastCrossingTime = 0U;
        _peakAmplitudeSum = 0.0f;
        _maxInput = -FLT_MAX;
        _minInput = FLT_MAX;
        _oscillationPeriod = 0.0f;
        _ultimateGain = 0.0f;
    } else {
        _tuneInProgress = false;
    }
}

/** @brief Sets manual mode output (0-100%). */
void AutoTunePID::setManualOutput(float output)
{
    _manualOutput = constrain(output, 0.0f, 100.0f);
}

/** @brief Sets override mode output. */
void AutoTunePID::setOverrideOutput(float output)
{
    _overrideOutput = constrain(output, _minOutput, _maxOutput);
}

/** @brief Sets tracking reference. */
void AutoTunePID::setTrackReference(float reference)
{
    _trackReference = reference;
}

/** @brief Sets tuning oscillation intensity. */
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
    default:
        _oscillationSteps = 10;
        break;
    }
}

/** @brief Sets exact number of tuning oscillations. */
void AutoTunePID::setOscillationSteps(int32_t steps)
{
    // BUG FIX #8: Add upper bound check to prevent device hang
    if (steps > 0 && steps <= 1000) {
        _oscillationSteps = steps;
    }
}

/** @brief Sets lambda for IMC/Lambda tuning. */
void AutoTunePID::setLambda(float lambda)
{
    // Validate lambda is positive and finite
    if (lambda > 0.0f && isfinite(lambda)) {
        _lambda = lambda;
    } else {
        _lambda = 0.5f;  // Safe default
    }
}

/** @brief Main update loop with strict timing. */
void AutoTunePID::update(float currentInput)
{
    const uint32_t now = millis();

    // BUG FIX: Prevent startup spike. Initialize state on first call.
    if (_lastUpdate == 0U) {
        _lastUpdate = now;
        _previousInput = currentInput;
        _input = currentInput;
        return;
    }

    // Maintain consistent sample time (100ms)
    if ((now - _lastUpdate) < 100U) {
        return;
    }

    float dt = static_cast<float>(now - _lastUpdate) / 1000.0f;
    _lastUpdate = now;

    // Call the deterministic core logic
    update(currentInput, dt);
}

/** @brief Deterministic processing loop for RTOS or fixed-interval ISRs. */
void AutoTunePID::update(float currentInput, float dt)
{
    // Guard against division by zero in derivative calculation
    if (dt < 0.001f) {
        dt = 0.001f;  // Minimum 1ms delta
    }

    // Apply input filtering if enabled
    if (_inputFilterEnabled && (_operationalMode != OperationalMode::Tune)) {
        currentInput = computeFilteredValue(currentInput, _inputFilteredValue, _inputFilterAlpha);
    }
    _input = currentInput;

    // Handle different operational modes
    if (_operationalMode == OperationalMode::Tune) {
        performAutoTune(currentInput);
    } else if (_operationalMode == OperationalMode::Manual) {
        // High-precision float-based formula for Manual mode
        _output = _minOutput + (_manualOutput / 100.0f) * (_maxOutput - _minOutput);
        _output = constrain(_output, _minOutput, _maxOutput);
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
        // Correct error direction branching based on persistent reverse mode flag
        if (_reverseMode) {
            _error = _input - _setpoint;
        } else {
            _error = _setpoint - _input;
        }
        return;
    } else {
        // Normal, Reverse, or Auto PID modes
        if (_operationalMode == OperationalMode::Reverse || (_operationalMode == OperationalMode::Auto && _reverseMode)) {
            _error = _input - _setpoint;
        } else {
            _error = _setpoint - _input;
        }

        // Numerical integration
        _integral += _error * dt;

        // Numerical differentiation (Derivative on Measurement)
        // Using -d(Input)/dt prevents derivative kick on setpoint changes
        _derivative = -(_input - _previousInput) / dt;
        
        computePID();
        applyAntiWindup();
        _previousInput = _input;
    }

    // Apply output filtering if enabled
    if (_outputFilterEnabled && (_operationalMode != OperationalMode::Tune)) {
        _output = computeFilteredValue(_output, _outputFilteredValue, _outputFilterAlpha);
    }
}

/** @brief Internal auto-tune relay logic. */
void AutoTunePID::performAutoTune(float currentInput)
{
    const uint32_t currentTime = millis();

    float highOutput;
    float lowOutput;
    
    // Calculate relay levels based on oscillation mode
    const float range = _maxOutput - _minOutput;
    const float center = (_maxOutput + _minOutput) / 2.0f;
    
    switch (_oscillationMode) {
    case OscillationMode::Normal:
        highOutput = _maxOutput;
        lowOutput = _minOutput;
        break;
    case OscillationMode::Half:
        highOutput = center + (range * 0.25f);
        lowOutput = center - (range * 0.25f);
        break;
    case OscillationMode::Mild:
        highOutput = center + (range * 0.125f);
        lowOutput = center - (range * 0.125f);
        break;
    default:
        highOutput = _maxOutput;
        lowOutput = _minOutput;
        break;
    }

    // Track peaks and valleys
    if (currentInput > _maxInput) { _maxInput = currentInput; }
    if (currentInput < _minInput) { _minInput = currentInput; }

    // Relay toggle on setpoint crossing
    bool crossed = false;
    if (_outputState && (currentInput > _setpoint)) {
        _outputState = false;
        crossed = true;
    } else if (!_outputState && (currentInput < _setpoint)) {
        _outputState = true;
        crossed = true;
    }

    _output = _outputState ? highOutput : lowOutput;

    if (crossed) {
        if (_halfCycleCount > 0) {
            // Calculate period from half-cycle
            const uint32_t duration = currentTime - _lastCrossingTime;
            
            // Guard against division by zero when crossings happen in same millisecond
            if (duration > 0U) {
                _oscillationPeriod = (2.0f * static_cast<float>(duration)) / 1000.0f;
            } else {
                _oscillationPeriod = 0.1f;  // Fallback minimum period
            }
            
            // Accumulate amplitude: a = (peak - valley) / 2
            const float currentAmplitude = (_maxInput - _minInput) / 2.0f;
            if (currentAmplitude > 0.001f) {
                _peakAmplitudeSum += currentAmplitude;
            }
        }
        
        _lastCrossingTime = currentTime;
        _halfCycleCount++;
        
        // Reset peak/valley tracking for next half-cycle
        _maxInput = -FLT_MAX;
        _minInput = FLT_MAX;

        // End of tuning cycle
        if (_halfCycleCount >= _oscillationSteps) {
            const float avgAmplitude = _peakAmplitudeSum / static_cast<float>(_halfCycleCount - 1);
            const float relayAmplitude = (highOutput - lowOutput) / 2.0f;
            
            if (avgAmplitude > 0.001f) {
                _ultimateGain = (4.0f * relayAmplitude) / (kPi * avgAmplitude);
            } else {
                _ultimateGain = 0.0f;
            }

            // Algorithm selection
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

            _halfCycleCount = 0;
            _peakAmplitudeSum = 0.0f;
            _tuneInProgress = false;
            _operationalMode = OperationalMode::Normal;
        }
    }
}

/** @brief Gains using Ziegler-Nichols Ultimate Period method. */
void AutoTunePID::calculateZieglerNicholsGains()
{
    // Guard against division by zero
    if (_oscillationPeriod > 0.001f) {
        // Standard ZN rules for PID: Kp = 0.6*Ku, Ti = 0.5*Tu, Td = 0.125*Tu
        _kp = 0.6f * _ultimateGain;
        _ki = (2.0f * _kp) / _oscillationPeriod;
        _kd = 0.125f * _kp * _oscillationPeriod;
    } else {
        _kp = _ki = _kd = 0.0f;
    }
}

/** @brief Gains using Cohen-Coon empirical rules. */
void AutoTunePID::calculateCohenCoonGains()
{
    // Guard against division by zero
    if (_oscillationPeriod > 0.001f) {
        // Estimating process gain K. For relay oscillation: a = (4*d*K)/(pi*sqrt(1+(wT)^2))
        _kp = 0.8f * _ultimateGain;
        _ki = _kp / (0.8f * _oscillationPeriod);
        _kd = 0.194f * _kp * _oscillationPeriod;
    } else {
        _kp = _ki = _kd = 0.0f;
    }
}

/** @brief Gains using Internal Model Control (IMC). */
void AutoTunePID::calculateIMCGains()
{
    // Estimate FOPDT parameters from relay test
    _processTimeConstant = 0.67f * _oscillationPeriod;
    _deadTime = 0.17f * _oscillationPeriod;
    
    float lambda = _lambda;
    if (lambda <= 0.0f) {
        lambda = 0.5f * _processTimeConstant;
    }

    // Guard against division by zero
    if ((_processTimeConstant > 0.001f) && ((lambda + _deadTime) > 0.001f)) {
        // IMC for FOPDT: Kp = T / (K*(lambda + L))
        _kp = _processTimeConstant / (lambda + _deadTime);
        _ki = _kp / _processTimeConstant;
        _kd = (_kp * _deadTime) / 2.0f;
    } else {
        _kp = _ki = _kd = 0.0f;
    }
}

/** @brief Gains using Tyreus-Luyben conservative rules. */
void AutoTunePID::calculateTyreusLuybenGains()
{
    // Guard against division by zero
    if (_oscillationPeriod > 0.001f) {
        // Tyreus-Luyben: Kp = Ku / 2.2, Ti = 2.2 * Tu, Td = Tu / 6.3
        _kp = _ultimateGain / 2.2f;
        _ki = _kp / (2.2f * _oscillationPeriod);
        _kd = (_kp * _oscillationPeriod) / 6.3f;
    } else {
        _kp = _ki = _kd = 0.0f;
    }
}

/** @brief Gains using Lambda Tuning (Closed-loop damping). */
void AutoTunePID::calculateLambdaTuningGains()
{
    _processTimeConstant = 0.67f * _oscillationPeriod;
    _deadTime = 0.17f * _oscillationPeriod;
    
    float currentLambda = _lambda;
    if (currentLambda <= 0.0f) {
        currentLambda = 0.5f * _processTimeConstant;
    }

    // Guard against division by zero
    if ((_processTimeConstant > 0.001f) && ((currentLambda + _deadTime) > 0.001f)) {
        // Kp = T / (K * (lambda + L))
        _kp = _processTimeConstant / (currentLambda + _deadTime);
        _ki = _kp / _processTimeConstant;
        _kd = 0.5f * _kp * _deadTime;
    } else {
        _kp = _ki = _kd = 0.0f;
    }
}

/** @brief Internal PID math. */
void AutoTunePID::computePID()
{
    const float P = _kp * _error;
    const float I = _ki * _integral;
    const float D = _kd * _derivative;

    _output = P + I + D;
    _output = constrain(_output, _minOutput, _maxOutput);
}

/** @brief Prevents integral windup based on threshold. */
void AutoTunePID::applyAntiWindup()
{
    if (_antiWindupEnabled && (fabsf(_integral) > _integralWindupThreshold)) {
        _integral = constrain(_integral, -_integralWindupThreshold, _integralWindupThreshold);
    }
}

/** @brief Generic exponential moving average filter. */
float AutoTunePID::computeFilteredValue(float input, float& filteredValue, float alpha) const
{
    // Bound checks for input stability
    if (!isfinite(input)) {
        return filteredValue;
    }

    // Clamp alpha to safe range
    const float safeAlpha = constrain(alpha, 0.0f, 1.0f);

    // Exponential moving average formula
    filteredValue = (safeAlpha * input) + ((1.0f - safeAlpha) * filteredValue);
    
    // Clamp filter output to prevented numerical overflow and maintain consistency with output range
    filteredValue = constrain(filteredValue, _minOutput, _maxOutput);
    
    return filteredValue;
}

} // namespace atp
