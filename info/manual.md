# Manual PID Tuning Guide

This guide explains how to manually tune PID (Proportional-Integral-Derivative) controller values to achieve optimal performance. The process is beginner-friendly and covers the fundamentals, step-by-step tuning, common troubleshooting, and tips for improvement.


---

**What is a PID Controller?**

A PID controller adjusts the output of a system (e.g., motor speed, temperature, or position) based on three factors:

1. Proportional (Kp): Corrects current errors.
2. Integral (Ki): Eliminates past errors by integrating them over time.
3. Derivative (Kd): Predicts future errors based on the rate of error change.

---

**When to Use PID Control**

Use a PID controller when:
    - You need precise control of a system (e.g., robotics, temperature control, or motor speed).
    - The system tends to overshoot or oscillate without damping.
    - Manual adjustments (e.g., ON/OFF or fixed speed) are insufficient.

---

## Step 1: Understand Your System

Before tuning, you must understand the system you're working with. For example:
    - Is it a slow system (e.g., temperature) or a fast system (e.g., motor position)?
    How does the system respond to changes? For instance:
    - Does it oscillate or overshoot?
    - Is it too slow to react?

Tools like graphs, logs, or an oscilloscope will help visualize the system's response.

---

## Step 2: Tuning the PID Values

Follow these steps to manually tune your PID controller. Start with Ki = 0 and Kd = 0, and focus on each parameter separately.

---

### 2.1 Tuning Kp (Proportional Gain)

1. Set Kp to a small value (e.g., 1) while keeping Ki = 0 and Kd = 0.
2. Gradually increase Kp until the system:
    - Approaches the target value (setpoint) but may overshoot slightly.
    - Begins to oscillate slightly around the setpoint without being unstable.
3. Stop increasing Kp when the system oscillates continuously. This is your "critical proportional gain" (Kp,crit).
4. Reduce Kp slightly from this value to dampen oscillations.

5. What happens if Kp is too high?
    - The system may oscillate heavily or become unstable. What happens if Kp is too low?
    - The system will react sluggishly to changes.

---

### 2.2 Tuning Ki (Integral Gain)

1. With a stable Kp, start increasing Ki gradually from 0.
2. Observe how the system corrects long-term (steady-state) errors:
    - A low Ki might not fully eliminate errors.
    - A high Ki might cause overshoot or instability.
3. Stop increasing Ki when the system:
    - Corrects steady-state errors effectively.
    - Has minimal oscillation or overshoot.

4. What happens if Ki is too high?
    - The system may oscillate or overshoot frequently. What happens if Ki is too low?
    - The system may have residual errors and fail to reach the target.

---

### 2.3 Tuning Kd (Derivative Gain)

1. With Kp and Ki set, start increasing Kd from 0.
2. Observe how the system responds to disturbances:
    - Kd helps to reduce oscillations and overshoot by predicting the future behavior of the error.
3. Stop increasing Kd when the system becomes:
    - Stable (minimal oscillations).
    - Responsive without being overly sensitive to noise.

4. What happens if Kd is too high?
    - The system may become slow or jittery due to noise amplification. What happens if Kd is too low?
    - The system may oscillate or overshoot due to insufficient damping.

---

## Step 3: Fine-Tune and Test

Once you’ve adjusted all three values (Kp, Ki, Kd):

1. Test the system with different setpoints or disturbances.
2. Monitor the response:
    - Does it reach the setpoint quickly?
    - Does it overshoot or oscillate?
    - How long does it take to stabilize?
3. Make small adjustments if needed:
    - Increase Kp for faster response.
    - Increase Ki to reduce steady-state error.
    - Increase Kd to dampen oscillations.

---

## Step 4: Optional Ziegler-Nichols Method

For a more structured approach:

1. Set Ki = 0 and Kd = 0.
2. Increase Kp until the system oscillates continuously (Kp,crit).
3. Measure the oscillation period (T,crit).
4. Use these formulas to calculate initial PID values:
    ```
    Kp = 0.6 × Kp,crit
    Ki = 2 × Kp / T,crit
    Kd = Kp × T,crit / 8
    ```
Test and refine these values as needed.

---

## Step 5: Visualize the Results

Use graphs or logs to monitor:
- Overshoot: How far the system exceeds the setpoint.
- Settling Time: How quickly the system stabilizes.
- Oscillations: Repeated overshooting before stabilization.

---

Common Issues and Fixes

---

**Tools and Tips**

1. Use a simulation tool (e.g., MATLAB, Simulink, or Python) to test PID tuning without damaging hardware.
2. Log or graph data to visualize the system’s performance.
3. Start with small increments when adjusting Ki and Kd.

---

Example System: DC Motor Speed Control

Here’s how you might tune a DC motor:

1. Increase Kp until the motor speed reaches the target quickly but oscillates.
2. Add Ki to ensure it holds the speed without drifting.
3. Add Kd to dampen any oscillations caused by sudden speed changes.

---

**Conclusion**

Manually tuning PID values is an iterative process that requires patience and observation. By following these steps, even a beginner can achieve an optimal balance of responsiveness, stability, and accuracy.
For advanced use, consider tools use the automatic tuning algorithms profided on the library.
