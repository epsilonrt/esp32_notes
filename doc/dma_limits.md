Yes, you can use DMA simultaneously for SPI display and ADC on ESP32 Arduino, but there are important caveats:

## The Short Answer

**Yes, it's possible** - the ESP32 has multiple DMA channels and controllers that can operate concurrently. However, you need to be careful about resource allocation and potential conflicts.

## Key Considerations

**DMA Channel Availability:**
- ESP32 has 13 DMA channels shared across peripherals
- Different peripherals can use different channels simultaneously
- SPI and ADC (I2S) typically use separate DMA channels, so they won't directly conflict

**TFT_eSPI with DMA:**
- TFT_eSPI library can use DMA for SPI transfers when configured properly
- It typically uses SPI DMA channels (often SPI3/HSPI or SPI2/VSPI)
- Make sure you enable DMA in your TFT_eSPI User_Setup.h

**ADC DMA Handling:**
- ESP32's ADC doesn't have native DMA support in the traditional sense
- For continuous ADC sampling with DMA, you typically use the **I2S peripheral** in ADC mode
- This uses I2S DMA channels, which are separate from SPI DMA

## Potential Issues

**Memory bandwidth:** Both operations accessing memory simultaneously could slow things down, but generally manageable for typical applications.

**CPU priority:** DMA interrupts from both peripherals need CPU attention. Usually fine unless you're doing extremely high-speed operations.

**IRAM limitations:** DMA ISR handlers should be in IRAM for best performance.

## Practical Example Pattern

```cpp
// SPI DMA for display (TFT_eSPI handles this)
tft.pushImage(...); // Non-blocking with DMA

// I2S in ADC mode for continuous sampling
i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
};
```

Both can run simultaneously without major issues for most applications.
