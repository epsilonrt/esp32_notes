# ESP-IDF's Native ADC Continuous (DMA) API

## Version Availability

The **`adc_continuous`** API was introduced in **ESP-IDF v4.4** (released in late 2021) and has been significantly improved in subsequent versions:

- **ESP-IDF v4.4**: Initial release of `adc_continuous` API
- **ESP-IDF v5.0+**: More stable and feature-complete
- **ESP-IDF v5.1+**: Additional improvements and bug fixes

## Arduino Framework Support

**Current Status:**
- **Arduino-ESP32 v2.x** (based on ESP-IDF 4.4): Partial support, API available but not well documented
- **Arduino-ESP32 v3.x** (based on ESP-IDF 5.1+): Better support for the continuous ADC API

You can use the native ESP-IDF API in Arduino by including the appropriate headers. The Arduino framework is essentially a wrapper around ESP-IDF, so you have access to the underlying APIs.

## How the New Interface Works

### Basic Architecture

The new API provides a dedicated continuous ADC mode with proper DMA support without using the I2S workaround:

```cpp
#include <Arduino.h>
#include "esp_adc/adc_continuous.h"

#define ADC_CHANNEL_INDEX ADC_CHANNEL_0   // Change to the GPIO you want to sample
#define ADC_ATTENUATION   ADC_ATTEN_DB_12 // 11 dB is deprecated in ESP-IDF 5, use 12 dB for ~3.3 V range
#define SAMPLE_RATE_HZ    20000
#define FRAME_LENGTH      256             // Bytes read per call (must be a multiple of SOC_ADC_DIGI_RESULT_BYTES)
#define NUM_CHANNELS      1

static adc_continuous_handle_t s_adc_handle = nullptr;

static void init_adc_continuous() {
  const adc_continuous_handle_cfg_t handle_cfg = {
    .max_store_buf_size = 1024,
    .conv_frame_size = FRAME_LENGTH,
  };
  ESP_ERROR_CHECK (adc_continuous_new_handle (&handle_cfg, &s_adc_handle));

  adc_digi_pattern_config_t pattern[NUM_CHANNELS] = {};
  pattern[0].atten = ADC_ATTENUATION;
  pattern[0].channel = ADC_CHANNEL_INDEX;
  pattern[0].unit = ADC_UNIT_1;
  pattern[0].bit_width = ADC_BITWIDTH_12;

  const adc_continuous_config_t adc_config = {
    .pattern_num = NUM_CHANNELS,
    .adc_pattern = pattern,
    .sample_freq_hz = SAMPLE_RATE_HZ,
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1, // Use TYPE2 on ESP32-C6/C5 if required
  };
  ESP_ERROR_CHECK (adc_continuous_config (s_adc_handle, &adc_config));
  ESP_ERROR_CHECK (adc_continuous_start (s_adc_handle));
}

void setup() {
  Serial.begin (115200);
  init_adc_continuous();
}

void loop() {
  uint8_t buffer[FRAME_LENGTH] = {0};
  uint32_t bytes_read = 0;

  const esp_err_t ret = adc_continuous_read (s_adc_handle, buffer, sizeof (buffer), &bytes_read, 100);

  if (ret == ESP_OK) {
    for (uint32_t offset = 0; offset + SOC_ADC_DIGI_RESULT_BYTES <= bytes_read; offset += SOC_ADC_DIGI_RESULT_BYTES) {
      const adc_digi_output_data_t *sample = reinterpret_cast<const adc_digi_output_data_t *> (&buffer[offset]);
      const uint32_t channel = sample->type1.channel;
      const uint32_t value = sample->type1.data;
      Serial.printf ("CH[%lu] = %lu\n", channel, value);
    }
  }
  else if (ret != ESP_ERR_TIMEOUT) {
    Serial.printf ("adc_continuous_read failed: %s\n", esp_err_to_name (ret));
  }

  delay (100);
}
```

## Complete Working Example with Multiple Channels

Here's a more complete example showing multi-channel sampling:

```cpp
#include "esp_adc/adc_continuous.h"

#define SAMPLE_RATE       20000  // Hz
#define READ_LEN          256
#define NUM_CHANNELS      2

static adc_continuous_handle_t handle = NULL;
static TaskHandle_t task_handle = NULL;

// Callback when conversion frame is done
static bool IRAM_ATTR adc_conv_done_cb(adc_continuous_handle_t handle, 
                                       const adc_continuous_evt_data_t *edata, 
                                       void *user_data) {
  BaseType_t mustYield = pdFALSE;
  vTaskNotifyGiveFromISR(task_handle, &mustYield);
  return (mustYield == pdTRUE);
}

void init_adc_continuous() {
  adc_continuous_handle_cfg_t adc_config = {
    .max_store_buf_size = 1024,
    .conv_frame_size = READ_LEN,
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));
  
  // Configure multiple channels
  adc_digi_pattern_config_t adc_pattern[NUM_CHANNELS];
  
  // Channel 0 - GPIO36
  adc_pattern[0].atten = ADC_ATTEN_DB_11;
  adc_pattern[0].channel = ADC_CHANNEL_0;
  adc_pattern[0].unit = ADC_UNIT_1;
  adc_pattern[0].bit_width = ADC_BITWIDTH_12;
  
  // Channel 3 - GPIO39
  adc_pattern[1].atten = ADC_ATTEN_DB_11;
  adc_pattern[1].channel = ADC_CHANNEL_3;
  adc_pattern[1].unit = ADC_UNIT_1;
  adc_pattern[1].bit_width = ADC_BITWIDTH_12;
  
  adc_continuous_config_t dig_cfg = {
    .pattern_num = NUM_CHANNELS,
    .adc_pattern = adc_pattern,
    .sample_freq_hz = SAMPLE_RATE,
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
  };
  
  ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
  
  // Register callback
  adc_continuous_evt_cbs_t cbs = {
    .on_conv_done = adc_conv_done_cb,
  };
  ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
  
  ESP_ERROR_CHECK(adc_continuous_start(handle));
}

void adc_task(void *param) {
  uint8_t result[READ_LEN] = {0};
  uint32_t ret_num = 0;
  
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    while (1) {
      esp_err_t ret = adc_continuous_read(handle, result, READ_LEN, &ret_num, 0);
      
      if (ret == ESP_OK) {
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
          adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
          
          uint32_t chan = p->type1.channel;
          uint32_t data = p->type1.data;
          
          // Process your data here
          Serial.printf("CH[%d]: %d\n", chan, data);
        }
      } else if (ret == ESP_ERR_TIMEOUT) {
        break;  // No more data in buffer
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  task_handle = xTaskGetCurrentTaskHandle();
  init_adc_continuous();
  
  xTaskCreate(adc_task, "adc_task", 4096, NULL, 2, &task_handle);
}

void loop() {
  delay(1000);
}
```

## How It Works Internally

### 1. **Hardware Flow**

```
ADC Peripheral → DMA Controller → Ring Buffer → Application
```

The new API manages:
- **DMA descriptors**: Automatically configured and linked
- **Ring buffer**: Circular buffer for continuous data flow
- **Interrupt handling**: Notifications when data is ready

### 2. **Data Format**

Each sample is encoded in a structure (TYPE1 format on ESP32):

```cpp
typedef struct {
    union {
        struct {
            uint16_t data:     12;  // ADC data (12 bits)
            uint16_t channel:   4;  // Channel number
        } type1;
        uint16_t val;
    };
} adc_digi_output_data_t;
```

### 3. **Conversion Modes**

- **ADC_CONV_SINGLE_UNIT_1**: Use only ADC1
- **ADC_CONV_SINGLE_UNIT_2**: Use only ADC2
- **ADC_CONV_BOTH_UNIT**: Use both ADC1 and ADC2 (time-multiplexed)
- **ADC_CONV_ALTER_UNIT**: Alternate between units

### 4. **Buffer Management**

```
DMA Buffer Chain:
[Buf1] → [Buf2] → [Buf3] → [Buf4] → [Buf1] (circular)
   ↓        ↓        ↓        ↓
Internal Ring Buffer (max_store_buf_size)
   ↓
Application reads via adc_continuous_read()
```

## Key Differences from I2S Method

| Feature | I2S-ADC Method | Native adc_continuous API |
|---------|----------------|---------------------------|
| API Complexity | More complex setup | Cleaner, purpose-built API |
| Multi-channel | Manual interleaving | Built-in support |
| Performance | Good | Optimized for ADC |
| WiFi conflict | Same issues | Same issues (ADC2) |
| Callback support | Manual | Built-in event callbacks |
| Buffer management | Manual | Automatic ring buffer |

## Advantages of New API

1. **Purpose-built**: Designed specifically for ADC, not repurposed from audio
2. **Multi-channel**: Native support for scanning multiple channels
3. **Cleaner code**: Less boilerplate configuration
4. **Better abstraction**: Don't need to understand I2S internals
5. **Event-driven**: Optional callbacks for efficient operation
6. **Automatic buffering**: Built-in ring buffer management

## Checking Arduino-ESP32 Version

To verify your Arduino-ESP32 version and ESP-IDF base:

```cpp
void setup() {
  Serial.begin(115200);
  Serial.printf("Arduino-ESP32 version: %s\n", ARDUINO_ESP32_GIT_DESC);
  Serial.printf("ESP-IDF version: %s\n", esp_get_idf_version());
}
```

If you're on an older Arduino-ESP32 version, you can still use the I2S method, or consider upgrading to v3.x for the best experience with the continuous ADC API.

## ESP32-C6 Specific Notes

The ESP32-C6 reuses the same `adc_continuous` API but its hardware differs from the original ESP32 family:

- Only one SAR ADC is present, so `ADC_UNIT_1` is the sole unit. Always select `ADC_CONV_SINGLE_UNIT_1`; `ADC_UNIT_2` and the alternating modes are not available.
- The digital controller exposes 8 external channels. The default channel macros map directly to the lower GPIO numbers:

```
ADC_CHANNEL_0 -> GPIO0
ADC_CHANNEL_1 -> GPIO1
ADC_CHANNEL_2 -> GPIO2
ADC_CHANNEL_3 -> GPIO3
ADC_CHANNEL_4 -> GPIO4
ADC_CHANNEL_5 -> GPIO5
ADC_CHANNEL_6 -> GPIO6
ADC_CHANNEL_7 -> GPIO7
```

- Some of these pins (for example GPIO0) are also strap pins or share functions with the USB-to-UART bridge. Plan your analog routing so you do not interfere with boot mode selection or other peripherals.
- The legacy I2S "ADC built-in" trick is not implemented on ESP32-C6, so `adc_continuous` (or the lower-level `adc_digi` driver) is the supported path for continuous DMA capture.
