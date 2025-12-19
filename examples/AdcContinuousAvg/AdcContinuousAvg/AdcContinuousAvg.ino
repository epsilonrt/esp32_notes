/*
 * Continuous ADC monitor for ESP32-C6.
 * - Captures ADC samples in continuous mode at 2 kHz and groups them in frames
 *   sized to cover two 50 Hz periods, ensuring stable statistics over the mains cycle.
 * - Applies ESP-IDF curve-fitting calibration (and falls back to raw scaling) so all
 *   reported voltages are in calibrated volts rather than ADC counts.
 * - Computes per-frame min/avg/max volts and prints them over Serial for quick
 *   validation of the analog front-end.
 * - Optionally enables the hardware IIR filter, controlled by the defines below, and
 *   drives the Type2 digital output format so channel metadata rides with each sample.
 */
#include <Arduino.h>
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_filter.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_err.h"

#define SAMPLE_RATE_HZ    10000 // see adc_continuous.md note (raised to keep IIR cutoff above 50 Hz)
#define SIGNAL_FREQ_HZ   50  // mains frequency
#define RMS_PERIOD_COUNT 2   // number of mains periods per frame
#define SAMPLES_PER_FRAME (SAMPLE_RATE_HZ / SIGNAL_FREQ_HZ * RMS_PERIOD_COUNT) // Samples per frame
#define FRAME_LENGTH      (SAMPLES_PER_FRAME * sizeof(adc_digi_output_data_t))  // Bytes per ADC read
#define NUM_CHANNELS      1   // Monitoring only one channel
#define ADC_CHANNEL_INDEX ADC_CHANNEL_0 // GPIO0 on ESP32-C6
#define ADC_ATTENUATION   ADC_ATTEN_DB_12 // Matches the calibration config
#define ADC_VREF_VOLTS    3.3   // Approximate full-scale with 12 dB attenuation, used if calibration is unavailable
#define ADC_IIR_FILTER_ENABLED    1  // Set to 1 to enable hardware IIR filter, 0 to disable
#define ADC_IIR_FILTER_COEFF      ADC_DIGI_IIR_FILTER_COEFF_8
#define DMA_BUFFER_MULTIPLIER 2  // Number of frames retained in the DMA ring buffer (limited to <= 4096 bytes)
#define MAX_STORE_BUF_SIZE    (FRAME_LENGTH * DMA_BUFFER_MULTIPLIER)  // Total DMA bytes

static_assert (MAX_STORE_BUF_SIZE <= 4096, "DMA buffer exceeds driver limit");

#if ADC_IIR_FILTER_ENABLED
static adc_iir_filter_handle_t s_adc_iir_handle = nullptr;
#endif
adc_continuous_handle_t s_adc_handle = nullptr;
adc_cali_handle_t s_adc_cali_handle = nullptr;

// ----------------------------------------------------------------------------
// Initialize ADC calibration
bool init_adc_calibration() {
  if (s_adc_cali_handle != nullptr) {
    return true;
  }

  adc_cali_curve_fitting_config_t cali_config = {
    .unit_id = ADC_UNIT_1,
    .atten = ADC_ATTENUATION,
    .bitwidth = ADC_BITWIDTH_12,
  };

  return adc_cali_create_scheme_curve_fitting (&cali_config, &s_adc_cali_handle) == ESP_OK;
}

// ----------------------------------------------------------------------------
// Convert raw ADC counts to voltage in volts
double raw_counts_to_volts (uint32_t raw_counts) {
  if (s_adc_cali_handle != nullptr) {
    int voltage_mv = 0;
    if (adc_cali_raw_to_voltage (s_adc_cali_handle, static_cast<int> (raw_counts), &voltage_mv) == ESP_OK) {
      return static_cast<double> (voltage_mv) / 1000.0;
    }
  }

  return static_cast<double> (raw_counts) * (ADC_VREF_VOLTS / 4096.0);
}

// ----------------------------------------------------------------------------
// Configure the continuous ADC engine for ADC unit 1 channel 0.
void init_adc_continuous() {
  // Configure the continuous ADC handle.
  adc_continuous_handle_cfg_t handle_cfg = {
    .max_store_buf_size = MAX_STORE_BUF_SIZE,
    .conv_frame_size = FRAME_LENGTH,
  };

  // Create the continuous ADC handle and check errors early.
  // The handle is the entry point for configuring, starting, reading, and stopping the engine.
  // It also stores the internal state of the continuous ADC engine.
  ESP_ERROR_CHECK (adc_continuous_new_handle (&handle_cfg, &s_adc_handle));

  // Configure ADC patterns, one entry per channel.
  adc_digi_pattern_config_t pattern[NUM_CHANNELS];

  // Channel 0: GPIO0 only.
  pattern[0].atten = ADC_ATTENUATION;      // GPIO, matched to calibration config
  pattern[0].channel = ADC_CHANNEL_INDEX;
  pattern[0].unit = ADC_UNIT_1; // Only ADC unit 1 is available on the C6.
  pattern[0].bit_width = ADC_BITWIDTH_12; // 12-bit resolution.

  // Configure the continuous ADC engine.
  const adc_continuous_config_t adc_config = {
    .pattern_num = NUM_CHANNELS,
    .adc_pattern = pattern,
    .sample_freq_hz = SAMPLE_RATE_HZ,
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2, // Required on ESP32-C6: includes channel info per sample
  };
  ESP_ERROR_CHECK (adc_continuous_config (s_adc_handle, &adc_config));

#if ADC_IIR_FILTER_ENABLED
  const adc_continuous_iir_filter_config_t filter_cfg = {
    .unit = ADC_UNIT_1,
    .channel = ADC_CHANNEL_INDEX,
    .coeff = ADC_IIR_FILTER_COEFF,
  };
  ESP_ERROR_CHECK (adc_new_continuous_iir_filter (s_adc_handle, &filter_cfg, &s_adc_iir_handle));
  ESP_ERROR_CHECK (adc_continuous_iir_filter_enable (s_adc_iir_handle));
#endif

  ESP_ERROR_CHECK (adc_continuous_start (s_adc_handle));

  if (!init_adc_calibration ()) {
    Serial.println ("ADC calibration not available, using raw scaling.");
  }
}

// ----------------------------------------------------------------------------
// Print min/avg/max statistics from the captured ADC frame
void print_stats (uint8_t *buffer, uint32_t length) {

  const adc_digi_output_data_t *samples = reinterpret_cast<const adc_digi_output_data_t *> (buffer);
  const uint32_t count = length / sizeof (adc_digi_output_data_t);

  if (count == 0) {
    return;
  }

  const uint32_t channel = samples[0].type2.channel;
  double sum_volts = 0.0;
  double min_volts = 0.0;
  double max_volts = 0.0;
  bool first_sample = true;

  for (uint32_t i = 0; i < count; ++i) {
    const uint32_t value = samples[i].type2.data;
    const double volts = raw_counts_to_volts (value);

    sum_volts += volts;

    if (first_sample) {
      min_volts = volts;
      max_volts = volts;
      first_sample = false;
    }
    else {
      min_volts = min (min_volts, volts);
      max_volts = max (max_volts, volts);
    }
  }

  const double avg_volts = sum_volts / static_cast<double> (count);

  Serial.printf ("CH[%lu] -> avg=%.3f V min=%.3f V max=%.3f V (n=%lu)\n",
                 channel,
                 avg_volts,
                 min_volts,
                 max_volts,
                 count);
}

void setup() {
  Serial.begin (115200);
  // Allow the serial port to open when flashing over USB.
  while (!Serial && millis() < 2000) {
    delay (10);
  }

  init_adc_continuous();
  Serial.printf ("ADC continuous started (rate %d Hz, frame %d bytes)\n", SAMPLE_RATE_HZ, FRAME_LENGTH);
}

void loop() {
  uint8_t result[FRAME_LENGTH] = {0};
  uint32_t ret_num = 0;

  const esp_err_t status = adc_continuous_read (s_adc_handle, result, FRAME_LENGTH, &ret_num, 100); // 100 ms timeout

  if (status == ESP_OK) {
    print_stats (result, ret_num);
  }
  else if (status != ESP_ERR_TIMEOUT) {
    Serial.printf ("adc_continuous_read failed: %s\n", esp_err_to_name (status));
    delay (1000);
  }

  delay (500);
}
