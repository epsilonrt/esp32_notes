# ADC avec DMA sur ESP32 Arduino

Cette note explique comment utiliser l'ADC avec DMA sur l'ESP32 et ce qui se passe en interne.

## Concept de base

Le DMA (Direct Memory Access) permet à l'ADC de transférer les données directement en mémoire sans intervention du CPU, ce qui est essentiel pour un échantillonnage continu à haute vitesse. Le CPU reste disponible pour d'autres tâches pendant que l'ADC capture les données en arrière-plan.

## Utilisation sous Arduino

Le framework ESP32 Arduino n'expose pas la fonctionnalité DMA via la simple fonction `analogRead()`. Pour utiliser l'ADC avec DMA, il faut s'appuyer sur l'API ESP-IDF :

```cpp
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define ADC_CHANNEL ADC1_CHANNEL_0  // GPIO36
#define SAMPLE_COUNT 1024
#define SAMPLE_RATE 20000  // Hz

uint16_t adc_buffer[SAMPLE_COUNT];

void setup() {
  Serial.begin(115200);
  
  // Configure l'ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);
  
  // Pour une lecture continue avec DMA, utiliser
  // l'astuce périphérique I2S ou la nouvelle API adc_continuous
}
```

## Utiliser le DMA I2S pour l'ADC (méthode courante)

L'ESP32 ré-utilise le périphérique I2S et son DMA pour lire l'ADC en continu :

```cpp
#include "driver/i2s.h"
#include "driver/adc.h"

#define I2S_NUM I2S_NUM_0
#define SAMPLE_RATE 10000
#define DMA_BUF_COUNT 4
#define DMA_BUF_LEN 1024

void setupADC_DMA() {
  // Configure I2S pour l'ADC
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
  
  // Traite les données - les 12 bits de poids fort portent la valeur ADC
  for(int i = 0; i < bytes_read/2; i++) {
    uint16_t adc_value = buffer[i] & 0xFFF;
    Serial.println(adc_value);
  }
}
```

## Fonctionnement interne

### 1. Architecture matérielle de l'ADC

L'ESP32 embarque deux ADC SAR (Successive Approximation Register) :
- **ADC1** : 8 canaux (GPIO 32-39)
- **ADC2** : 10 canaux (GPIO 0, 2, 4, 12-15, 25-27)

Chaque ADC convertit la tension d'entrée en la comparant à une référence interne via une recherche binaire.

### 2. Contrôleur DMA

L'ESP32 propose 13 canaux DMA partagés entre les périphériques. Une fois configuré :

1. **Chaîne de descripteurs DMA** : liste chaînée en mémoire, chaque entrée référence
   - Un buffer de stockage,
   - La taille du buffer,
   - Un pointeur vers le descripteur suivant.

2. **Flux d'opération DMA** :
   ```
   Échantillons ADC -> FIFO ADC -> Contrôleur DMA -> Buffer RAM -> Lecture CPU quand prêt
   ```

### 3. Passerelle I2S-ADC

Comme les premières versions de l'IDF ne proposaient pas de mode DMA continu natif pour l'ADC, le périphérique I2S est ré-utilisé :

- Le périphérique I2S bénéficie d'un DMA robuste prévu pour l'audio,
- L'ESP32 autorise le routage interne de la sortie ADC vers l'entrée I2S,
- Le DMA de l'I2S pousse les échantillons vers les buffers mémoire,
- Le tout fonctionne à une fréquence d'échantillonnage réglable avec un minimum d'intervention CPU.

### 4. Déroulement pas à pas

1. **Phase de configuration**
   - Réglage de la résolution (9-12 bits) et de l'atténuation,
   - Configuration d'I2S en mode ADC intégré,
   - Allocation de 2 à 8 buffers DMA pour le ping-pong,
   - Chaînage des descripteurs DMA.

2. **Phase d'exécution**
   - Un timer déclenche la conversion ADC,
   - L'ADC convertit l'analogique en numérique,
   - Le résultat arrive dans la FIFO I2S,
   - Le contrôleur DMA transfère automatiquement vers le buffer courant,
   - Une fois plein, le DMA passe au buffer suivant,
   - Une interruption ou un drapeau avertit le CPU,
   - Le CPU traite le buffer rempli pendant que le DMA charge le suivant.

3. **Disposition mémoire**
   ```
   [Buffer 1] <-> DMA en remplissage
   [Buffer 2] <-> Traitement CPU
   [Buffer 3] <-> Prêt à remplir
   [Buffer 4] <-> Prêt à remplir
   ```

## Avantages du DMA

1. **Efficacité CPU** : le CPU n'a pas besoin de scruter chaque échantillon,
2. **Fréquences élevées** : plus de 100 k échantillons par seconde,
3. **Temporisation déterministe** : intervalles définis par le matériel,
4. **Acquisition continue** : pas de trou dans les données.

## Points importants

- ADC2 est indisponible quand le WiFi est actif (matériel partagé),
- La fréquence pratique maximale tourne autour de 200 kHz,
- Les buffers DMA doivent se trouver dans une RAM compatible DMA (pas PSRAM sur certains modèles),
- Les données arrivent par blocs, pas échantillon par échantillon,
- Une calibration est conseillée pour optimiser la précision.

Cette architecture permet à l'ESP32 de capturer des données en continu à haute vitesse tout en laissant le CPU libre pour du traitement ou de la communication.
