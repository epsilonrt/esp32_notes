/*
  ESP32-C6 continuous ADC monitor that captures frames via DMA and reports RMS voltage snapshots.
  Each frame spans RMS_PERIOD_COUNT mains periods; RMS can optionally target only the AC component
  (DC mean removed) to highlight ripple. When ADC_IIR_FILTER_ENABLED is set the ESP-IDF hardware IIR
  filter smooths samples before they reach DMA, giving a single-pole low-pass response with cutoff
  determined by SAMPLE_RATE_HZ and ADC_IIR_FILTER_COEFF. Output can be Teleplot-friendly or plain text
  depending on TELEPLOT_ENABLED.
*/
#include <Arduino.h>
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_filter.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_err.h"

#define ADC_CHANNEL_INDEX ADC_CHANNEL_1 // GPIO1 on ESP32-C6
#define ADC_ATTENUATION   ADC_ATTEN_DB_12 // Matches the calibration config
#define RMS_REMOVE_MEAN    false  // Set to true to compute AC RMS (removing DC mean), false for total RMS

#define SIGNAL_FREQ_HZ    50    // mains frequency
#define SAMPLE_RATE_HZ    10000 // see adc_continuous.md and iir_filter.md note (raised to keep IIR cutoff above 50 Hz)

#define ADC_IIR_FILTER_ENABLED    1  // Set to 1 to enable hardware IIR filter, 0 to disable
#define ADC_IIR_FILTER_COEFF      ADC_DIGI_IIR_FILTER_COEFF_8

#define RMS_PERIOD_COUNT 2   // number of mains periods per frame
#define SAMPLES_PER_FRAME (SAMPLE_RATE_HZ / SIGNAL_FREQ_HZ * RMS_PERIOD_COUNT) // Samples per frame
#define FRAME_LENGTH      (SAMPLES_PER_FRAME * sizeof(adc_digi_output_data_t))  // Bytes per ADC read
#define NUM_CHANNELS      1   // Monitoring only one channel
#define ADC_VREF_VOLTS    3.3   // Approximate full-scale with 12 dB attenuation, used if calibration is unavailable
#define DMA_BUFFER_MULTIPLIER 2  // Number of frames retained in the DMA ring buffer (limited to <= 4096 bytes)
#define MAX_STORE_BUF_SIZE    (FRAME_LENGTH * DMA_BUFFER_MULTIPLIER)  // Total DMA bytes

#define TELEPLOT_ENABLED 0 // Set to 1 to enable Teleplot output (requires Teleplot extension and setup)

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
// Configure the continuous ADC engine for ADC unit 1 channel 1.
void init_adc_continuous() {
  // Configure the continuous ADC handle.
  adc_continuous_handle_cfg_t handle_cfg = {
    .max_store_buf_size = MAX_STORE_BUF_SIZE,
    .conv_frame_size = FRAME_LENGTH,
  };

  ESP_ERROR_CHECK (adc_continuous_new_handle (&handle_cfg, &s_adc_handle));

  adc_digi_pattern_config_t pattern[NUM_CHANNELS];

  pattern[0].atten = ADC_ATTENUATION;
  pattern[0].channel = ADC_CHANNEL_INDEX;
  pattern[0].unit = ADC_UNIT_1;
  pattern[0].bit_width = ADC_BITWIDTH_12;

  const adc_continuous_config_t adc_config = {
    .pattern_num = NUM_CHANNELS,
    .adc_pattern = pattern,
    .sample_freq_hz = SAMPLE_RATE_HZ,
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
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
// Compute and print the RMS voltage from the ADC samples in the buffer.
void print_rms (uint8_t *buffer, uint32_t length, bool remove_mean = false) {

  const adc_digi_output_data_t *samples = reinterpret_cast<const adc_digi_output_data_t *> (buffer);
  const uint32_t count = length / sizeof (adc_digi_output_data_t);

  if (count == 0) {
    return;
  }

  const uint32_t channel = samples[0].type2.channel;
  double sum_volts = 0.0;
  double sum_square_volts = 0.0;

  for (uint32_t i = 0; i < count; ++i) {
    const uint32_t value = samples[i].type2.data;
    const double volts = raw_counts_to_volts (value);

    sum_volts += volts;
    sum_square_volts += volts * volts;
  }

  double mean_volts = 0.0;
  double rms_volts = 0.0;

  if (remove_mean) {
    mean_volts = sum_volts / static_cast<double> (count);
    const double mean_square = mean_volts * mean_volts;
    const double ac_power = max (0.0, (sum_square_volts / static_cast<double> (count)) - mean_square);
    rms_volts = sqrt (ac_power);
  }
  else {
    rms_volts = sqrt (sum_square_volts / static_cast<double> (count));
  }

  #if TELEPLOT_ENABLED
  Serial.printf (">%s%lu:%.3f\n",
                 remove_mean ? "AC" : "RMS",
                 channel,
                 rms_volts);
  #else
  Serial.printf ("%.3f\n", rms_volts);
  #endif

}

void setup() {
  Serial.begin (115200);
  while (!Serial && millis() < 2000) {
    delay (10);
  }

  init_adc_continuous();
  Serial.printf ("ADC continuous RMS started (rate %d Hz, frame %d bytes)\n", SAMPLE_RATE_HZ, FRAME_LENGTH);
}

void loop() {
  uint8_t result[FRAME_LENGTH] = {0};
  uint32_t ret_num = 0;

  const esp_err_t status = adc_continuous_read (s_adc_handle, result, FRAME_LENGTH, &ret_num, 100);

  if (status == ESP_OK) {
    print_rms (result, ret_num, RMS_REMOVE_MEAN);
  }
  else if (status != ESP_ERR_TIMEOUT) {
    Serial.printf ("adc_continuous_read failed: %s\n", esp_err_to_name (status));
    delay (1000);
  }

  delay (500);
}
