# Continuous ADC — Sampling Frequency

In ESP32-C6 continuous ADC mode, the sampling frequency is set by the time divider inside the digital controller.

$$F_{\text{sample}} = \frac{F_{\text{digi\_con}}}{2 \cdot \text{interval}}$$

with $F_{\text{digi\_con}} = 5\,\text{MHz}$ and $30 \leq \text{interval} \leq 4095$. Each conversion consumes two clock cycles, and the interval states how many times this pair of cycles repeats before launching the next measurement.

**2 kHz example**

- Target: $F_{\text{sample}} = 2\,\text{kHz}$
- Computation: $\text{interval} = \frac{5\,000\,000}{2 \times 2\,000} = 1250$
- Outcome: the ADC inserts exactly 1250 digital-clock periods between successive samples, guaranteeing the requested frequency.