#include <Arduino.h>
#include "esp_adc/adc_continuous.h"
#include "esp_err.h"

#define SAMPLE_RATE_HZ    2000
#define FRAME_LENGTH      256
#define NUM_CHANNELS      2

adc_continuous_handle_t s_adc_handle = nullptr;

// Configure le moteur ADC continu avec deux canaux du C6.
void init_adc_continuous() {

  // Configuration du handle ADC continu.
  adc_continuous_handle_cfg_t handle_cfg = {
    .max_store_buf_size = 1024,
    .conv_frame_size = FRAME_LENGTH,
  };

  // Créé le handle ADC continu en vérifiant les erreurs, le cas échéant.
  // Le handle est le point d'entrée pour toutes les autres opérations ADC continues.
  // Il est utilisé pour configurer, démarrer, lire et arrêter le moteur ADC continu.
  // Il stocke également l'état interne du moteur ADC continu.
  ESP_ERROR_CHECK (adc_continuous_new_handle (&handle_cfg, &s_adc_handle));

  // Configuration des patterns ADC, un pattern correspond à un canal.
  adc_digi_pattern_config_t pattern[NUM_CHANNELS];

  // Canal 0 : GPIO0.
  pattern[0].atten = ADC_ATTEN_DB_12;      // GPIO0
  pattern[0].channel = ADC_CHANNEL_0;
  pattern[0].unit = ADC_UNIT_1; // Uniquement l'unité 1 est utilisée sur le C6.
  pattern[0].bit_width = ADC_BITWIDTH_12; // Résolution 12 bits.

  // Canal 3 : GPIO3
  pattern[1].atten = ADC_ATTEN_DB_12;      // GPIO3
  pattern[1].channel = ADC_CHANNEL_3;
  pattern[1].unit = ADC_UNIT_1;
  pattern[1].bit_width = ADC_BITWIDTH_12;

  // Configuration du moteur ADC continu.
  const adc_continuous_config_t adc_config = {
    .pattern_num = NUM_CHANNELS,
    .adc_pattern = pattern,
    .sample_freq_hz = SAMPLE_RATE_HZ,
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1, // 4bits MSB = N° de canal, 12bits LSB = valeur ADC
  };
  ESP_ERROR_CHECK (adc_continuous_config (s_adc_handle, &adc_config));

  ESP_ERROR_CHECK (adc_continuous_start (s_adc_handle));
}

void print_samples (uint8_t *buffer, uint32_t length) {

  const uint16_t *samples = reinterpret_cast<const uint16_t *> (buffer);
  const uint32_t count = length / sizeof (uint16_t);

  for (uint32_t i = 0; i < count; ++i) {
    const uint16_t raw = samples[i];
    const uint32_t channel = (raw >> 12) & 0x0F;  // Bits 15..12
    const uint32_t value = raw & 0x0FFF;          // Bits 11..0
    Serial.printf ("CH[%lu] -> %lu\n", channel, value);
  }
}

void setup() {
  Serial.begin (115200);
  // Laisse le temps au port série de s'ouvrir lors de la programmation USB.
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
    print_samples (result, ret_num);
  }
  else if (status != ESP_ERR_TIMEOUT) {
    Serial.printf ("adc_continuous_read failed: %s\n", esp_err_to_name (status));
    delay (1000);
  }

  delay (50);
}
