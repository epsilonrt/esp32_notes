# ADC with DMA on ESP32 Arduino

Let me explain how to use ADC with DMA on the ESP32 and what's happening under the hood.

## Basic Concept

DMA (Direct Memory Access) allows the ADC to transfer data directly to memory without CPU intervention, which is crucial for high-speed continuous sampling. This frees up the CPU for other tasks while the ADC continuously captures data in the background.

## How to Use It in Arduino

The ESP32 Arduino framework doesn't expose DMA functionality directly in the simple `analogRead()` function. To use ADC with DMA, you need to use the ESP-IDF API:

```cpp
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define ADC_CHANNEL ADC1_CHANNEL_0  // GPIO36
#define SAMPLE_COUNT 1024
#define SAMPLE_RATE 20000  // Hz

uint16_t adc_buffer[SAMPLE_COUNT];

void setup() {
  Serial.begin(115200);
  
  // Configure ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);
  
  // For continuous reading with DMA, you'll need to use
  // the I2S peripheral trick or the newer adc_continuous API
}
```

## Using I2S DMA for ADC (Common Method)

The ESP32 has a clever trick: you can use the I2S peripheral's DMA to read ADC data continuously:

```cpp
#include "driver/i2s.h"
#include "driver/adc.h"

#define I2S_NUM I2S_NUM_0
#define SAMPLE_RATE 10000
#define DMA_BUF_COUNT 4
#define DMA_BUF_LEN 1024

void setupADC_DMA() {
  // Configure I2S for ADC
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_LSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  
  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0);
  i2s_adc_enable(I2S_NUM);
}

void readADC_DMA() {
  uint16_t buffer[DMA_BUF_LEN];
  size_t bytes_read;
  
  i2s_read(I2S_NUM, buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);
  
  // Process data - note: upper 12 bits contain ADC value
  for(int i = 0; i < bytes_read/2; i++) {
    uint16_t adc_value = buffer[i] & 0xFFF;
    Serial.println(adc_value);
  }
}
```

## How It Works Internally

### 1. **ADC Hardware Architecture**

The ESP32 has two SAR (Successive Approximation Register) ADCs:
- **ADC1**: 8 channels (GPIOs 32-39)
- **ADC2**: 10 channels (GPIOs 0, 2, 4, 12-15, 25-27)

Each ADC performs conversion by comparing the input voltage against an internal reference through a binary search process.

### 2. **DMA Controller**

The ESP32 has 13 DMA channels shared among peripherals. When configured:

1. **DMA descriptor chain**: A linked list of descriptors in memory, each pointing to:
   - A buffer where data will be stored
   - Size of the buffer
   - Pointer to the next descriptor

2. **DMA operation flow**:
   ```
   ADC samples → ADC FIFO → DMA controller → RAM buffer → CPU reads when ready
   ```

### 3. **I2S-ADC Bridge**

Since the ESP32's ADC doesn't have native continuous mode DMA in earlier IDF versions, the I2S peripheral is repurposed:

- The I2S peripheral has robust DMA support for audio streaming
- ESP32 allows routing ADC output to I2S input internally
- I2S DMA then transfers ADC samples to memory buffers
- This happens at a configurable sample rate with minimal CPU involvement

### 4. **The Process Step-by-Step**

1. **Setup Phase**:
   - Configure ADC resolution (9-12 bits) and attenuation
   - Configure I2S with ADC built-in mode
   - Allocate DMA buffers (typically 2-8 buffers for ping-pong operation)
   - Link DMA descriptors in a chain

2. **Runtime Phase**:
   - Timer triggers ADC conversion at sample rate
   - ADC converts analog to digital
   - Result placed in I2S FIFO
   - DMA controller automatically transfers from FIFO to current buffer
   - When buffer full, DMA switches to next buffer (ping-pong)
   - Interrupt/flag notifies CPU that buffer is ready
   - CPU processes filled buffer while DMA fills the other

3. **Memory Layout**:
   ```
   [Buffer 1] ←→ DMA filling
   [Buffer 2] ←→ CPU processing
   [Buffer 3] ←→ Ready to fill
   [Buffer 4] ←→ Ready to fill
   ```

## Advantages of DMA

1. **CPU efficiency**: CPU doesn't poll or wait for each sample
2. **High sample rates**: Can achieve 100k+ samples/second
3. **Deterministic timing**: Hardware-controlled sampling intervals
4. **Continuous operation**: No gaps in data collection

## Important Notes

- ADC2 cannot be used with WiFi active (they share hardware)
- Maximum practical sample rate is around 200 kHz
- DMA buffers must be in DMA-capable RAM (not PSRAM on some models)
- Data arrives in chunks, not single samples
- Some calibration recommended for accuracy

This architecture allows the ESP32 to perform continuous, high-speed data acquisition while leaving the CPU free for processing and communication tasks.
