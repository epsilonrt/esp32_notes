# Digital Filtering Overview

## General Principle
A digital filter transforms a sequence of samples into a new sequence whose spectral content better matches a target specification. The filter detects unwanted frequency components—noise, aliases, out-of-band energy—and attenuates them while preserving the desired band. Because the system is discrete, all operations occur at a fixed sampling frequency $F_s$ and use difference equations rather than differential equations.

## Families of Filters
- **FIR (Finite Impulse Response)**: The output is a weighted sum of a finite number of past input samples. FIR filters are inherently stable and can be designed with linear phase, but steep transitions require many taps (more memory/CPU).
- **IIR (Infinite Impulse Response)**: The output reuses previous outputs as well as current inputs. This recursion provides sharp transitions with few coefficients but introduces phase distortion and must be kept stable (poles constrained to the unit circle).
- **Other types**: Adaptive filters (LMS, RLS), Kalman filters, multi-rate/decimation filters, etc., all tailored to specific signal-processing goals.

## IIR Filters in Detail
An IIR filter obeys a difference equation such as:
$$y[n] = \sum_{k=0}^{M} b_k x[n-k] - \sum_{k=1}^{N} a_k y[n-k]$$
It mirrors the structure of analog filters (Butterworth, Chebyshev, Bessel) and achieves steep frequency response with minimal coefficients. Benefits include compact implementations and steep roll-offs; trade-offs are nonlinear phase, sensitivity to coefficient quantization, and potential instability if poles leave the unit circle.

### Hardware IIR on ESP32-C6
The ESP32-C6 continuous ADC driver exposes a hardware IIR filter that implements an exponential moving average (EMA):
$$y[n] = y[n-1] + \frac{x[n] - y[n-1]}{K}$$
Here $K \in \{2,4,8,16,32,64\}$. Smaller $K$ values yield a wider bandwidth (less smoothing); larger $K$ values yield stronger smoothing and more latency. The effective cutoff frequency scales with the sampling rate:
$$f_c \approx \frac{F_s}{2\pi K}$$
So doubling $F_s$ doubles the bandwidth, while increasing $K$ narrows it.

### Effect on 50 Hz Measurements
When monitoring a 50 Hz sinusoidal signal, enabling the hardware IIR filter reduces high-frequency noise or spikes before calculating statistics (avg/min/max or RMS). The result is a cleaner amplitude estimate at the cost of a small delay (~$K/2$ samples) and a softer response to sudden transients. For RMS computations, the filter often improves stability because noise spikes are damped before squaring. Note that the cutoff must sit above the 50 Hz tone to avoid amplitude loss: with $F_s = 2\,\text{kHz}$ and $K = 8$, the cutoff is only ~$40$ Hz, so the sinus is partially attenuated. Raising the sampling rate to $F_s \approx 2\pi K f_c \approx 2.5\,\text{kHz}$ keeps $f_c \approx 50$ Hz while preserving the EMA behavior.
