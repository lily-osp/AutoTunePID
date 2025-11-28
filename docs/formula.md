## **1. Ziegler-Nichols**

### Formula:

$ K_p = 0.6 \cdot K_u $

$ K_i = \frac{1.2 \cdot K_p}{T_u} $

$ K_d = 0.075 \cdot K_p \cdot T_u $

---

## **2. Cohen-Coon**

### Formula:

$ K_p = 0.8 \cdot K_u $

$ K_i = \frac{K_p}{0.8 \cdot T_u} $

$ K_d = 0.194 \cdot K_p \cdot T_u $

---

## **3. Internal Model Control (IMC)**

### Formula:

$ K_p = \frac{T}{\lambda + L} $

$ K_i = \frac{K_p}{T} $

$ K_d = \frac{K_p \cdot L}{2} $

Where:
- $ T $: Process time constant
- $ L $: Dead time
- $ \lambda $: Tuning parameter (configurable)

---

## **4. Tyreus-Luyben**

### Formula:

$ K_p = 0.45 \cdot K_u $

$ K_i = \frac{K_p}{2.2 \cdot T_u} $

$ K_d = 0 \quad \text{(No derivative action for stability)} $

---

## **5. Lambda Tuning (CLD)**

### Formula:

$ K_p = \frac{T}{\lambda + L} $

$ K_i = \frac{K_p}{T} $

$ K_d = 0.5 \cdot K_p \cdot L $

Where:
- $ T $: Process time constant
- $ L $: Dead time
- $ \lambda $: Tuning parameter (configurable)

---

### **Key Notes**:

1. **Ultimate Gain Calculation**: $ K_u = \frac{4d}{\pi a} $ where $ d $ is the relay amplitude and $ a $ is the oscillation amplitude.
2. **Process Parameter Estimation**: $ T \approx 0.67 \cdot T_u $, $ L \approx 0.17 \cdot T_u $ where $ T_u $ is the oscillation period.
3. **Time-Based Integration**: The integral term uses proper time steps: $ \int e(t) \, dt \approx \sum e(t) \cdot \Delta t $
4. **Derivative Calculation**: Uses time-based derivative: $ \frac{de(t)}{dt} \approx \frac{e(t) - e(t-1)}{\Delta t} $
5. All formulas are implemented in the **AutoTunePID library** for automatic calculation.

### **Implementation Details**

The AutoTunePID library implements these formulas with the following enhancements:

- **Proper Time Integration**: Uses actual time steps instead of fixed multipliers
- **Anti-Windup Protection**: Prevents integral windup with configurable thresholds
- **Signal Filtering**: Exponential moving averages for input/output smoothing
- **Oscillation Modes**: Normal, Half, and Mild oscillation amplitudes for auto-tuning
- **Operational Modes**: Normal, Reverse, Hold, Preserve, Tune, and Auto modes

---

### **Conclusion**

These tuning methods provide a comprehensive set of tools for PID controller design. The **Ziegler-Nichols** and **Cohen-Coon** methods offer simple, effective tuning for most applications, while **IMC** and **Lambda Tuning** provide advanced control for systems with significant dead time. The **Tyreus-Luyben** method ensures stability-focused tuning. All formulas are mathematically correct and implemented with numerical stability considerations in the AutoTunePID library.

---
