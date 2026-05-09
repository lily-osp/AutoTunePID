/**
 * @file AutoTunePID.h
 * @brief Header file for the AutoTunePID library.
 * @details Implements a robust, AUTOSAR C++14 compliant PID controller with multiple auto-tuning algorithms.
 */

#ifndef AUTOTUNEPID_H
#define AUTOTUNEPID_H

#include <Arduino.h>
#include <stdint.h>
#include <cfloat>
#include <cmath>

/**
 * @namespace atp
 * @brief Namespace for the AutoTunePID library.
 */
namespace atp {

/**
 * @enum TuningMethod
 * @brief Enumeration for different PID tuning algorithms.
 * @details Mapped to uint8_t for memory efficiency and AUTOSAR compliance.
 */
enum class TuningMethod : uint8_t {
    ZieglerNichols, /**< Ziegler-Nichols tuning method (Classic) */
    CohenCoon,      /**< Cohen-Coon tuning method (Better for laggy systems) */
    IMC,            /**< Internal Model Control tuning method (Robust) */
    TyreusLuyben,   /**< Tyreus-Luyben tuning method (Conservative) */
    LambdaTuning,   /**< Lambda Tuning (CLD) method (Smooth response) */
    Manual          /**< Manual tuning method (Direct gain setting) */
};

/**
 * @brief Backward compatibility for tuning methods.
 */
constexpr auto ZieglerNichols = TuningMethod::ZieglerNichols;
constexpr auto CohenCoon = TuningMethod::CohenCoon;
constexpr auto IMC = TuningMethod::IMC;
constexpr auto TyreusLuyben = TuningMethod::TyreusLuyben;
constexpr auto LambdaTuning = TuningMethod::LambdaTuning;
constexpr auto Manual = TuningMethod::Manual;

/**
 * @enum OperationalMode
 * @brief Enumeration for controller operational modes.
 */
enum class OperationalMode : uint8_t {
    Normal,   /**< Normal PID operation (Heating/Direct) */
    Reverse,  /**< Reverse PID operation (Cooling/Indirect) */
    Manual,   /**< Manual mode: direct output control */
    Override, /**< Override mode: fixed emergency output */
    Track,    /**< Track mode: output follows a reference signal */
    Hold,     /**< Hold mode: maintain output, reset states */
    Preserve, /**< Preserve mode: minimal calculations, keep responsive */
    Tune,     /**< Tune mode: perform auto-tuning process */
    Auto      /**< Auto mode: automatically determine best mode */
};

/**
 * @enum OscillationMode
 * @brief Defines the amplitude of oscillations during auto-tuning.
 */
enum class OscillationMode : uint8_t {
    Normal, /**< Full oscillation (MaxOutput - MinOutput) */
    Half,   /**< Half oscillation (Center +/- 25% range) */
    Mild    /**< Mild oscillation (Center +/- 12.5% range) */
};

/**
 * @class AutoTunePID
 * @brief A safety-critical PID controller with built-in auto-tuning.
 * @details Compliant with AUTOSAR C++14 standards.
 */
class AutoTunePID {
public:
    /**
     * @brief Constructor for the PID controller.
     * @param minOutput The minimum allowed output value.
     * @param maxOutput The maximum allowed output value.
     * @param method Initial tuning method (default: Ziegler-Nichols).
     */
    explicit AutoTunePID(float minOutput, float maxOutput, TuningMethod method = TuningMethod::ZieglerNichols);

    // --- Configuration Methods ---

    /**
     * @brief Sets the desired target value.
     * @param setpoint The target value to reach.
     */
    void setSetpoint(float setpoint);

    /**
     * @brief Changes the auto-tuning method.
     * @param method The tuning method to use.
     */
    void setTuningMethod(TuningMethod method);

    /**
     * @brief Manually sets the PID gains.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     */
    void setManualGains(float kp, float ki, float kd);

    /**
     * @brief Enables and configures the input filter.
     * @param alpha Filter coefficient (0.01 - 1.0).
     */
    void enableInputFilter(float alpha);

    /**
     * @brief Enables and configures the output filter.
     * @param alpha Filter coefficient (0.01 - 1.0).
     */
    void enableOutputFilter(float alpha);

    /**
     * @brief Configures anti-windup protection.
     * @param enable Whether to enable protection.
     * @param threshold Integral limit factor (default: 0.8).
     */
    void enableAntiWindup(bool enable, float threshold = 0.8f);

    /**
     * @brief Switches the controller's operational mode.
     * @param mode The desired operational mode.
     */
    void setOperationalMode(OperationalMode mode);

    /**
     * @brief Sets the output value for Manual mode.
     * @param output Desired output (0 - 100%).
     */
    void setManualOutput(float output);

    /**
     * @brief Sets the output value for Override mode.
     * @param output Fixed output value.
     */
    void setOverrideOutput(float output);

    /**
     * @brief Sets the reference signal for Track mode.
     * @param reference The value to follow.
     */
    void setTrackReference(float reference);

    /**
     * @brief Sets the auto-tuning oscillation range.
     * @param mode The desired oscillation mode.
     */
    void setOscillationMode(OscillationMode mode);

    /**
     * @brief Sets the number of oscillations for tuning.
     * @param steps Number of steps (must be > 0 and <= 1000).
     */
    void setOscillationSteps(int32_t steps);

    /**
     * @brief Sets the lambda parameter for Lambda/IMC tuning.
     * @param lambda The desired closed-loop time constant (must be > 0).
     */
    void setLambda(float lambda);

    // --- Runtime Methods ---

    /**
     * @brief Main processing loop for the PID controller.
     * @param currentInput The latest process measurement.
     */
    void update(float currentInput);

    /** @return Current controller output. */
    float getOutput() const { return _output; }
    /** @return Proportional gain (Kp). */
    float getKp() const { return _kp; }
    /** @return Integral gain (Ki). */
    float getKi() const { return _ki; }
    /** @return Derivative gain (Kd). */
    float getKd() const { return _kd; }
    /** @return Calculated Ultimate Gain (Ku). */
    float getKu() const { return _ultimateGain; }
    /** @return Calculated Oscillation Period (Tu). */
    float getTu() const { return _oscillationPeriod; }
    /** @return Current setpoint. */
    float getSetpoint() const { return _setpoint; }
    /** @return Current operational mode. */
    OperationalMode getOperationalMode() const { return _operationalMode; }
    /** @return Current lambda parameter. */
    float getLambda() const { return _lambda; }

private:
    // PID Internal Computation
    void computePID();
    void applyAntiWindup();
    float computeFilteredValue(float input, float& filteredValue, float alpha) const;

    // Auto-tuning Internal Methods
    void performAutoTune(float currentInput);
    void calculateZieglerNicholsGains();
    void calculateCohenCoonGains();
    void calculateIMCGains();
    void calculateTyreusLuybenGains();
    void calculateLambdaTuningGains();

    // Configuration (Const)
    const float _minOutput; /**< Lower bound of output */
    const float _maxOutput; /**< Upper bound of output */

    // Configuration (Mutable)
    TuningMethod _method;           /**< Selected tuning method */
    OperationalMode _operationalMode; /**< Current operational mode */
    OscillationMode _oscillationMode; /**< Auto-tuning oscillation scope */
    int32_t _oscillationSteps;      /**< Number of tuning oscillations */
    float _setpoint;                /**< Target value */
    float _lambda;                  /**< Lambda parameter for IMC/Lambda tuning */

    // Operational parameters
    float _manualOutput;   /**< User-defined output in Manual mode */
    float _overrideOutput; /**< Fixed output in Override mode */
    float _trackReference; /**< Target reference in Track mode */

    // PID Gains and State
    float _kp;            /**< Proportional gain */
    float _ki;            /**< Integral gain */
    float _kd;            /**< Derivative gain */
    float _error;         /**< Current tracking error */
    float _previousError; /**< Error from last update */
    float _integral;      /**< Accumulated integral term */
    float _derivative;    /**< Current derivative term */
    float _output;        /**< Final computed output */
    float _input;         /**< Last process measurement */

    // Safety and Filtering
    bool _antiWindupEnabled;        /**< Flag for anti-windup status */
    float _integralWindupThreshold; /**< Limit for integral term */

    // Tuning Metrics
    uint32_t _lastUpdate;    /**< Millis() of last update */
    float _ultimateGain;     /**< Ku calculated from relay test */
    float _oscillationPeriod; /**< Tu in seconds from relay test */
    float _maxInput;         /**< Peak input during oscillation */
    float _minInput;         /**< Valley input during oscillation */
    uint32_t _lastPeakTime;  /**< Time of last peak for period calculation */
    bool _tuneInProgress;    /**< Flag to track if tuning is active (for static reset) */

    // Derived Process Parameters
    float _processTimeConstant; /**< Estimated system time constant (T) */
    float _deadTime;            /**< Estimated system dead time (L) */
    float _integralTime;        /**< Calculated Ti */
    float _derivativeTime;      /**< Calculated Td */

    // Filtering Infrastructure
    bool _inputFilterEnabled;   /**< Input smoothing enabled */
    bool _outputFilterEnabled;  /**< Output smoothing enabled */
    float _inputFilteredValue;  /**< State of input filter */
    float _outputFilteredValue; /**< State of output filter */
    float _inputFilterAlpha;    /**< Input filter responsiveness */
    float _outputFilterAlpha;   /**< Output filter responsiveness */

    // Constants
    static constexpr float kPi = 3.14159265f; /**< PI constant for calculations */
};

} // namespace atp

#endif
