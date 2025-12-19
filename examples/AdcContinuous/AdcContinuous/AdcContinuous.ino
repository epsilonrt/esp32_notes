/*
  Arduino example for continuous ADC sampling on ESP32 variants (original, S2, S3, C3, C6, H2).
  It configures a single ADC channel, streams samples via DMA, and prints min/max/average per frame.
  The code relies only on public ESP-IDF APIs, so it runs unchanged on every Arduino-ESP32 target.
*/
#include <Arduino.h>
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_err.h"

#define ADC_CHANNEL_INDEX ADC_CHANNEL_0  // GPIO number depends on the selected channel (0 -> GPIO0)
#define ADC_ATTENUATION   ADC_ATTEN_DB_12 // ADC input attenuation (impacts full-scale voltage)
#define SAMPLE_RATE_HZ    2000

#define FRAME_LENGTH      256 // Each sample is 4 bytes, so 256 bytes = 64 samples
#define NUM_CHANNELS      1
#define ADC_VREF_VOLTS    3.3f  // Effective full-scale with 12 dB attenuation, used if calibration is unavailable

adc_continuous_handle_t s_adc_handle = nullptr;
adc_cali_handle_t s_adc_cali_handle = nullptr;
bool s_adc_cali_enabled = false;

// ----------------------------------------------------------------------------
// Initialize ADC calibration
bool init_adc_calibration() {
  if (s_adc_cali_enabled) {
    return true;
  }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  // If curve fitting calibration is supported, use it.
  // It generally provides better accuracy than line fitting.
  adc_cali_curve_fitting_config_t cali_config = {
    .unit_id = ADC_UNIT_1,
    .atten = ADC_ATTENUATION,
    .bitwidth = ADC_BITWIDTH_12,
  };
  if (adc_cali_create_scheme_curve_fitting (&cali_config, &s_adc_cali_handle) == ESP_OK) {
    s_adc_cali_enabled = true;
  }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  // Fall back to line fitting if curve fitting is not supported.
  // Line fitting is less accurate but more widely supported.

  adc_cali_line_fitting_config_t cali_config = {
    .unit_id = ADC_UNIT_1,
    .atten = ADC_ATTENUATION,
    .bitwidth = ADC_BITWIDTH_12,
  };
  if (adc_cali_create_scheme_line_fitting (&cali_config, &s_adc_cali_handle) == ESP_OK) {
    s_adc_cali_enabled = true;
  }
#else
  s_adc_cali_enabled = false;
#endif

  return s_adc_cali_enabled;
}

// ----------------------------------------------------------------------------
// Convert raw ADC counts to voltage in volts
float raw_counts_to_volts (uint32_t raw_counts) {
  if (s_adc_cali_enabled && s_adc_cali_handle != nullptr) {
    int voltage_mv = 0;
    if (adc_cali_raw_to_voltage (s_adc_cali_handle, static_cast<int> (raw_counts), &voltage_mv) == ESP_OK) {
      return voltage_mv / 1000.0f;
    }
  }

  return raw_counts * (ADC_VREF_VOLTS / 4096.0f);
}

// ----------------------------------------------------------------------------
// Configure the continuous ADC engine for the selected channel on ADC unit 1.
void init_adc_continuous() {

  // Configure the continuous ADC handle.
  adc_continuous_handle_cfg_t handle_cfg = {
    .max_store_buf_size = 1024,
    .conv_frame_size = FRAME_LENGTH,
  };

  // Create the continuous ADC handle and check errors early.
  // The handle is the entry point for configuring, starting, reading, and stopping the engine.
  // It also stores the internal state of the continuous ADC engine.
  ESP_ERROR_CHECK (adc_continuous_new_handle (&handle_cfg, &s_adc_handle));

  // Configure ADC patterns, one entry per channel.
  adc_digi_pattern_config_t pattern[NUM_CHANNELS];

  // Single channel sampling with the configured attenuation.
  pattern[0].atten = ADC_ATTENUATION;
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
  uint32_t min_value = UINT32_MAX;
  uint32_t max_value = 0;
  uint64_t sum = 0;

  for (uint32_t i = 0; i < count; ++i) {
    const uint32_t value = samples[i].type2.data;
    min_value = min (min_value, value);
    max_value = max (max_value, value);
    sum += value;
  }

  const float avg_counts = static_cast<float> (sum) / static_cast<float> (count);
  const float avg_volts = raw_counts_to_volts (static_cast<uint32_t> (avg_counts + 0.5f));
  const float min_volts = raw_counts_to_volts (min_value);
  const float max_volts = raw_counts_to_volts (max_value);

  Serial.printf ("CH[%lu] -> avg=%.1f (%.3f V) min=%lu (%.3f V) max=%lu (%.3f V) (n=%lu)\n",
                 channel, avg_counts, avg_volts,
                 min_value, min_volts,
                 max_value, max_volts,
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
