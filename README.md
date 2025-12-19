# ESP32 ADC Notes

This workspace collects field notes, experiments, and reference material about ESP32 analog-to-digital conversion (ADC) in continuous mode. The repo is organized so you can jump straight to the documentation you need or to a ready-to-build PlatformIO example.

## Documentation (doc/)
- `adc_continuous.md` – overview of continuous ADC capture and configuration hints.
- `ADC_dma*.md` – several French/English drafts describing DMA-driven ADC, buffer limits, and the latest findings.
- `iir_filter*.md` – notes on applying IIR filtering to ADC streams (both EN/FR versions).
- `parameters_opt_limits.md` – practical parameter limits observed during testing.
- `dma_limits.md` – measured DMA throughput ceilings and troubleshooting tips.

## Examples (examples/)
Each subfolder is a standalone PlatformIO project targeting ESP32 boards:
- `AdcContinuous` – minimal continuous ADC capture sketch.
- `AdcContinuousAvg` – continuous capture with moving-average post-processing.
- `AdcContinuousRms` – continuous capture with RMS computation.

Mirror copies of `AdcContinuousAvg` and `AdcContinuousRms` exist at the repo root for quick opening in VS Code when the `examples/` workspace is already in use.

## Getting Started
1. Install PlatformIO IDE or CLI.
2. Open any project folder (for example `examples/AdcContinuousAvg`).
3. Build and upload with `platformio run -t upload`.

## Contributions
Feel free to document additional experiments, especially around DMA tuning, filtering strategies, or calibration techniques. Keep notes concise so they remain easy to reference during development.

