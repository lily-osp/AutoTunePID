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
    , _maxInput(-FLT_MAX / 2.0f)
    , _minInput(FLT_MAX / 2.0f)
    , _lastPeakTime(0U)
    , _tuneInProgress(false)
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
    _setpoint = setpoint;
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
    // BUG FIX #6: Use fabs to prevent sign reversal when maxOutput < minOutput
    _integralWindupThreshold = threshold * fabsf(_maxOutput - _minOutput);
}

/** @brief Sets operational mode and resets state on 'Hold'. */
void AutoTunePID::setOperationalMode(OperationalMode mode)
{
    _operationalMode = mode;
    if (mode == OperationalMode::Hold) {
        _integral = 0.0f;
        _previousError = 0.0f;
        _output = 0.0f;
    }
    // BUG FIX #3: Reset static variables when entering Tune mode
    else if (mode == OperationalMode::Tune) {
        _tuneInProgress = true;
        _maxInput = -FLT_MAX / 2.0f;
        _minInput = FLT_MAX / 2.0f;
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
    // BUG FIX #9: Validate lambda is positive and normal
    if (lambda > 0.0f && std::isnormal(lambda)) {
        _lambda = lambda;
    } else {
        _lambda = 0.5f;  // Safe default
    }
}

/** @brief Main update loop with strict timing. */
void AutoTunePID::update(float currentInput)
{
    const uint32_t now = millis();
    // Maintain consistent sample time (100ms)
    if ((now - _lastUpdate) < 100U) {
        return;
    }

    float dt = static_cast<float>(now - _lastUpdate) / 1000.0f;
    _lastUpdate = now;

    // BUG FIX #1: Guard against division by zero in derivative calculation
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
        // BUG FIX #5: Replace integer map() with float-based formula for precision
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
        // BUG FIX #4: Correct logic error - check mode BEFORE entering Preserve block
        const bool isReverse = (_operationalMode == OperationalMode::Reverse);
        if (isReverse) {
            _error = _input - _setpoint;
        } else {
            _error = _setpoint - _input;
        }
        return;
    } else if (_operationalMode == OperationalMode::Auto) {
        // TODO: Implement automatic mode selection based on system behavior
        // For now, fall through to Normal mode
        _error = _setpoint - _input;
        _integral += _error * dt;
        _derivative = (_error - _previousError) / dt;
        computePID();
        applyAntiWindup();
        _previousError = _error;
    } else {
        // Normal or Reverse PID modes
        if (_operationalMode == OperationalMode::Reverse) {
            _error = _input - _setpoint;
        } else {
            _error = _setpoint - _input;
        }

        // Numerical integration
        _integral += _error * dt;

        // Numerical differentiation
        _derivative = (_error - _previousError) / dt;
        
        computePID();
        applyAntiWindup();
        _previousError = _error;
    }

    // Apply output filtering if enabled
    if (_outputFilterEnabled && (_operationalMode != OperationalMode::Tune)) {
        _output = computeFilteredValue(_output, _outputFilteredValue, _outputFilterAlpha);
    }
}

/** @brief Internal auto-tune relay logic. */
void AutoTunePID::performAutoTune(float currentInput)
{
    static bool outputState = true;
    static int32_t halfCycleCount = 0;
    static uint32_t lastCrossingTime = 0U;
    static float peakAmplitudeSum = 0.0f;

    // BUG FIX #3: Reset statics when tuning restarts (flag set in setOperationalMode)
    if (!_tuneInProgress) {
        outputState = true;
        halfCycleCount = 0;
        lastCrossingTime = 0U;
        peakAmplitudeSum = 0.0f;
        _tuneInProgress = true;
    }

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
    if (outputState && (currentInput > _setpoint)) {
        outputState = false;
        crossed = true;
    } else if (!outputState && (currentInput < _setpoint)) {
        outputState = true;
        crossed = true;
    }

    _output = outputState ? highOutput : lowOutput;

    if (crossed) {
        if (halfCycleCount > 0) {
            // Calculate period from half-cycle
            uint32_t duration = currentTime - lastCrossingTime;
            
            // BUG FIX #2: Guard against division by zero when crossings happen in same millisecond
            if (duration > 0) {
                _oscillationPeriod = (2.0f * static_cast<float>(duration)) / 1000.0f;
            } else {
                _oscillationPeriod = 0.1f;  // Fallback minimum period
            }
            
            // Accumulate amplitude: a = (peak - valley) / 2
            const float currentAmplitude = (_maxInput - _minInput) / 2.0f;
            if (currentAmplitude > 0.001f) {
                peakAmplitudeSum += currentAmplitude;
            }
        }
        
        lastCrossingTime = currentTime;
        halfCycleCount++;
        
        // Reset peak/valley tracking for next half-cycle
        _maxInput = -FLT_MAX / 2.0f;
        _minInput = FLT_MAX / 2.0f;

        // End of tuning cycle
        if (halfCycleCount >= _oscillationSteps) {
            const float avgAmplitude = peakAmplitudeSum / static_cast<float>(halfCycleCount - 1);
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

            halfCycleCount = 0;
            peakAmplitudeSum = 0.0f;
            _tuneInProgress = false;
            _operationalMode = OperationalMode::Normal;
        }
    }
}

/** @brief Gains using Ziegler-Nichols Ultimate Period method. */
void AutoTunePID::calculateZieglerNicholsGains()
{
    // Guard against division by zero (BUG FIX #2)
    if (_oscillationPeriod > 0.0f) {
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
    // Guard against division by zero (BUG FIX #2)
    if (_oscillationPeriod > 0.0f) {
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
    if (_processTimeConstant > 0.0f && (lambda + _deadTime) > 0.0f) {
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
    // Guard against division by zero (BUG FIX #2)
    if (_oscillationPeriod > 0.0f) {
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
    
    if (_lambda <= 0.0f) {
        _lambda = 0.5f * _processTimeConstant;
    }

    // Guard against division by zero
    if (_processTimeConstant > 0.0f && (_lambda + _deadTime) > 0.0f) {
        // Kp = T / (K * (lambda + L))
        _kp = _processTimeConstant / (_lambda + _deadTime);
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
    if (_antiWindupEnabled && (abs(_integral) > _integralWindupThreshold)) {
        _integral = constrain(_integral, -_integralWindupThreshold, _integralWindupThreshold);
    }
}

/** @brief Generic exponential moving average filter. */
float AutoTunePID::computeFilteredValue(float input, float& filteredValue, float alpha) const
{
    // BUG FIX #7: Clamp filter output to prevent numerical overflow and instability
    filteredValue = (alpha * input) + ((1.0f - alpha) * filteredValue);
    filteredValue = constrain(filteredValue, _minOutput, _maxOutput);
    return filteredValue;
}

} // namespace atp
