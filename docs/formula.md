## **1. Lambda Tuning (CLD)**

### Formula:

$ K_p = \frac{T}{K(\lambda + L)} $

$ K_i = \frac{K_p}{T} = \frac{1}{K(\lambda + L)} $

$ K_d = K_p \cdot 0.5L = \frac{0.5L \cdot T}{K(\lambda + L)} $

---

## **2. Internal Model Control (IMC)**

### Formula:

$ K_p = \frac{T}{K(\tau_c + L)} $

$ K_i = \frac{K_p}{T} = \frac{1}{K(\tau_c + L)} $

$ K_d = K_p \cdot 0.5L = \frac{0.5L \cdot T}{K(\tau_c + L)} $

---

## **3. Tyreus-Luyben**

### Formula:

$ K_p = 0.31K_u $

$ K_i = \frac{K_p}{2.2T_u} = \frac{0.31K_u}{2.2T_u} = 0.141K_u / T_u $

$ K_d = 0 \quad \text{(Derivative action is usually omitted in Tyreus-Luyben for stability)} $

---

## **4. Ziegler-Nichols**

### Formula:

$ K_p = 0.6K_u $

$ K_i = \frac{K_p}{0.5T_u} = \frac{0.6K_u}{0.5T_u} = 1.2K_u / T_u $

$ K_d = K_p \cdot 0.125T_u = 0.6K_u \cdot 0.125T_u = 0.075K_u T_u $

---

## **5. Cohen-Coon**

### Formula:

$ K_p = \frac{1}{K} \left( 1.35 + \frac{T}{L} \right) $

$ K_i = \frac{K_p}{\frac{2.5L}{1 + 0.35(T/L)}} = \frac{K_p \cdot (1 + 0.35(T/L))}{2.5L} $

$ K_d = K_p \cdot 0.37L $

---

### **Key Notes**:

1. $ K_i $ is derived as $ K_i = K_p / T_i $, so integral time $ T_i $ is replaced in terms of $ K_p $.
2. $ K_d $ is derived as $ K_d = K_p \cdot T_d $, replacing $ T_d $ in terms of $ K_p $.
3. These formulas are ready for direct use in your **AutoTunePID library** to calculate $ K_p, K_i, K_d $.

### **Automation Workflow**

1. **Induce Oscillations**:
   
   - Use a proportional controller or relay to induce oscillations in the system.
   - Record:
     - $ K_u $: Ultimate gain (oscillation gain).
     - $ T_u $: Oscillation period.

2. **Estimate Process Parameters (if needed)**:
   
   - $ K, T, L $: Derived using $ K_u, T_u $.

3. **Calculate PID Gains**:
   
   - Use the appropriate method (Lambda, IMC, Tyreus-Luyben, Ziegler-Nichols, or Cohen-Coon).

4. **Implement the Controller**:
   
   - Apply the tuned $ K_p $, $ T_i $, and $ T_d $ values in your PID controller.

---

### **Conclusion**

The **Lambda Tuning (CLD)** method provides a robust approach for systems with significant dead time, while **IMC**, **Tyreus-Luyben**, **Ziegler-Nichols**, and **Cohen-Coon** offer additional options for different system dynamics. These formulas can be implemented programmatically in your **AutoTunePID library** to achieve optimal PID tuning.

---
