#include "sensor.h"

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

// DHT11: GPIO4 (bidirectional, 1-wire)
constexpr gpio_num_t kDht11Pin = GPIO_NUM_27;

// KS0033 NTC thermistor: GPIO34 = ADC1_CH6 (ADC1, safe with Wi-Fi)
constexpr adc_unit_t    kKs0033AdcUnit    = ADC_UNIT_1;
constexpr adc_channel_t kKs0033AdcChannel = ADC_CHANNEL_6;
constexpr adc_atten_t   kKs0033Atten      = ADC_ATTEN_DB_12;
// NTC parameters: 10 kΩ at 25 °C, Beta = 3950
// KS0033 onboard divider: VCC → NTC → S(out) → 10 kΩ → GND
// (no external resistor — module carries its own 10 kΩ)
constexpr float kNtcR0   = 20000.0f;
constexpr float kNtcT0   = 298.15f;   // 25 °C in Kelvin
constexpr float kNtcBeta = 3950.0f;

// Cytron moisture sensor A0: GPIO32 = ADC1_CH4
// Higher voltage = drier, so pct is inverted
constexpr adc_unit_t    kMoistureAdcUnit    = ADC_UNIT_1;
constexpr adc_channel_t kMoistureAdcChannel = ADC_CHANNEL_4;
constexpr adc_atten_t   kMoistureAtten      = ADC_ATTEN_DB_12;

// Light sensor (LDR): GPIO35 = ADC1_CH7 (ADC1, safe with Wi-Fi)
// Voltage divider: 3.3V → LDR → GPIO35 → 10 kΩ → GND
// Higher voltage = brighter light
constexpr adc_unit_t    kLightAdcUnit    = ADC_UNIT_1;
constexpr adc_channel_t kLightAdcChannel = ADC_CHANNEL_7;
constexpr adc_atten_t   kLightAtten      = ADC_ATTEN_DB_12;

const char *TAG = "sensor";

adc_oneshot_unit_handle_t g_adc1_handle    = nullptr;

adc_cali_handle_t g_ks0033_cali    = nullptr;
bool              g_ks0033_cali_ok = false;

adc_cali_handle_t g_moisture_cali    = nullptr;
bool              g_moisture_cali_ok = false;

adc_cali_handle_t g_light_cali    = nullptr;
bool              g_light_cali_ok = false;

static esp_err_t dht11_wait_level(int expected, int64_t timeout_us) {
    const int64_t deadline = esp_timer_get_time() + timeout_us;
    while (gpio_get_level(kDht11Pin) != expected) {
        if (esp_timer_get_time() >= deadline) {
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}

static void busy_wait_us(int64_t us) {
    const int64_t deadline = esp_timer_get_time() + us;
    while (esp_timer_get_time() < deadline) {}
}

static esp_err_t make_cali(adc_channel_t channel, adc_atten_t atten,
                            adc_cali_handle_t *out_handle, bool *out_ok) {
    *out_ok = false;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg = {};
    cfg.unit_id  = kLightAdcUnit;   // both channels are on ADC1
    cfg.chan     = channel;
    cfg.atten    = atten;
    cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cfg, out_handle);
    *out_ok = (ret == ESP_OK);
    if (!*out_ok) {
        ESP_LOGW(TAG, "ADC calibration unavailable for channel %d: %s", channel, esp_err_to_name(ret));
    }
#endif
    return ESP_OK;
}

}  // namespace

esp_err_t SensorManager::init() {
    ESP_RETURN_ON_ERROR(init_dht11(),    TAG, "DHT11 init failed");
    ESP_RETURN_ON_ERROR(init_ks0033(),   TAG, "KS0033 init failed");
    ESP_RETURN_ON_ERROR(init_moisture(), TAG, "moisture sensor init failed");
    ESP_RETURN_ON_ERROR(init_light(),    TAG, "light sensor init failed");
    return ESP_OK;
}

esp_err_t SensorManager::read(SensorData &out_data) {
    memset(&out_data, 0, sizeof(out_data));
    esp_err_t result = ESP_OK;

    if (read_dht11(out_data.dht11_temperature_c, out_data.dht11_humidity_pct) == ESP_OK) {
        out_data.dht11_ok = true;
    } else {
        result = ESP_FAIL;
    }

    if (read_ks0033(out_data.ks0033_temperature_c) == ESP_OK) {
        out_data.ks0033_ok = true;
    } else {
        result = ESP_FAIL;
    }

    if (read_moisture(out_data.moisture_raw, out_data.moisture_mv, out_data.moisture_pct) == ESP_OK) {
        out_data.moisture_ok = true;
    } else {
        result = ESP_FAIL;
    }

    if (read_light(out_data.light_raw, out_data.light_mv, out_data.light_pct) == ESP_OK) {
        out_data.light_ok = true;
    } else {
        result = ESP_FAIL;
    }

    return result;
}

esp_err_t SensorManager::init_dht11() {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << kDht11Pin);
    cfg.mode         = GPIO_MODE_INPUT;
    cfg.pull_up_en   = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type    = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "DHT11 GPIO config failed");

    // DHT11 needs ≥1 s after power-on before first read
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "DHT11 initialized on GPIO%d", kDht11Pin);
    return ESP_OK;
}

esp_err_t SensorManager::init_ks0033() {
    // Both KS0033 and light sensor share ADC1 — create the unit once here
    if (g_adc1_handle == nullptr) {
        adc_oneshot_unit_init_cfg_t unit_cfg = {};
        unit_cfg.unit_id  = ADC_UNIT_1;
        unit_cfg.clk_src  = ADC_RTC_CLK_SRC_DEFAULT;
        unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
        ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &g_adc1_handle), TAG, "ADC1 unit init failed");
    }

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten    = kKs0033Atten;
    chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_RETURN_ON_ERROR(
        adc_oneshot_config_channel(g_adc1_handle, kKs0033AdcChannel, &chan_cfg),
        TAG, "KS0033 channel config failed"
    );

    make_cali(kKs0033AdcChannel, kKs0033Atten, &g_ks0033_cali, &g_ks0033_cali_ok);

    ESP_LOGI(TAG, "KS0033 initialized on GPIO34 (ADC1_CH6)");
    return ESP_OK;
}

esp_err_t SensorManager::init_moisture() {
    if (g_adc1_handle == nullptr) {
        adc_oneshot_unit_init_cfg_t unit_cfg = {};
        unit_cfg.unit_id  = ADC_UNIT_1;
        unit_cfg.clk_src  = ADC_RTC_CLK_SRC_DEFAULT;
        unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
        ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &g_adc1_handle), TAG, "ADC1 unit init failed");
    }

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten    = kMoistureAtten;
    chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_RETURN_ON_ERROR(
        adc_oneshot_config_channel(g_adc1_handle, kMoistureAdcChannel, &chan_cfg),
        TAG, "moisture channel config failed"
    );

    make_cali(kMoistureAdcChannel, kMoistureAtten, &g_moisture_cali, &g_moisture_cali_ok);

    ESP_LOGI(TAG, "Moisture sensor initialized on GPIO32 (ADC1_CH4)");
    return ESP_OK;
}

esp_err_t SensorManager::init_light() {
    if (g_adc1_handle == nullptr) {
        adc_oneshot_unit_init_cfg_t unit_cfg = {};
        unit_cfg.unit_id  = ADC_UNIT_1;
        unit_cfg.clk_src  = ADC_RTC_CLK_SRC_DEFAULT;
        unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
        ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &g_adc1_handle), TAG, "ADC1 unit init failed");
    }

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten    = kLightAtten;
    chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_RETURN_ON_ERROR(
        adc_oneshot_config_channel(g_adc1_handle, kLightAdcChannel, &chan_cfg),
        TAG, "light sensor channel config failed"
    );

    make_cali(kLightAdcChannel, kLightAtten, &g_light_cali, &g_light_cali_ok);

    ESP_LOGI(TAG, "Light sensor initialized on GPIO35 (ADC1_CH7)");
    return ESP_OK;
}

esp_err_t SensorManager::read_dht11(float &temperature_c, float &humidity_pct) {
    // Send start signal: pull low ≥18 ms then release
    ESP_RETURN_ON_ERROR(gpio_set_direction(kDht11Pin, GPIO_MODE_OUTPUT), TAG, "dht11 gpio out failed");
    gpio_set_level(kDht11Pin, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(kDht11Pin, 1);
    ESP_RETURN_ON_ERROR(gpio_set_direction(kDht11Pin, GPIO_MODE_INPUT), TAG, "dht11 gpio in failed");

    portDISABLE_INTERRUPTS();

    // DHT11 response: 80 µs low then 80 µs high
    if (dht11_wait_level(0, 150) != ESP_OK) { portENABLE_INTERRUPTS(); return ESP_ERR_TIMEOUT; }
    if (dht11_wait_level(1, 150) != ESP_OK) { portENABLE_INTERRUPTS(); return ESP_ERR_TIMEOUT; }
    if (dht11_wait_level(0, 150) != ESP_OK) { portENABLE_INTERRUPTS(); return ESP_ERR_TIMEOUT; }

    // Read 40 bits: each = ~50 µs low + 26 µs ('0') or 70 µs ('1') high
    uint8_t data[5] = {0};
    for (int i = 0; i < 40; i++) {
        if (dht11_wait_level(1, 100) != ESP_OK) { portENABLE_INTERRUPTS(); return ESP_ERR_TIMEOUT; }
        busy_wait_us(40);  // sample at 40 µs: still high → '1', already low → '0'
        data[i / 8] = static_cast<uint8_t>(data[i / 8] << 1);
        if (gpio_get_level(kDht11Pin)) {
            data[i / 8] |= 1u;
        }
        if (dht11_wait_level(0, 100) != ESP_OK) { portENABLE_INTERRUPTS(); return ESP_ERR_TIMEOUT; }
    }

    portENABLE_INTERRUPTS();

    const uint8_t checksum = static_cast<uint8_t>(data[0] + data[1] + data[2] + data[3]);
    if (checksum != data[4]) {
        ESP_LOGW(TAG, "DHT11 checksum mismatch: calc=0x%02X recv=0x%02X", checksum, data[4]);
        return ESP_ERR_INVALID_CRC;
    }

    humidity_pct  = static_cast<float>(data[0]) + static_cast<float>(data[1]) * 0.1f;
    temperature_c = static_cast<float>(data[2]) + static_cast<float>(data[3]) * 0.1f;
    return ESP_OK;
}

esp_err_t SensorManager::read_ks0033(float &temperature_c) {
    if (g_adc1_handle == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw = 0;
    ESP_RETURN_ON_ERROR(adc_oneshot_read(g_adc1_handle, kKs0033AdcChannel, &raw), TAG, "KS0033 ADC read failed");

    int mv = 0;
    if (g_ks0033_cali_ok && g_ks0033_cali != nullptr) {
        ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(g_ks0033_cali, raw, &mv), TAG, "KS0033 cali failed");
    } else {
        mv = (raw * 3300) / 4095;
    }

    // KS0033 divider: VCC → NTC → S → 10 kΩ → GND
    // V_out = V_cc × R_fixed / (R_fixed + R_ntc)  →  R_ntc = R_fixed × (V_cc − V_out) / V_out
    const float v_out = static_cast<float>(mv);
    const float v_cc  = 3300.0f;
    if (v_out <= 0.0f || v_out >= v_cc) {
        ESP_LOGW(TAG, "KS0033 voltage out of range: %d mV", mv);
        return ESP_ERR_INVALID_RESPONSE;
    }
    const float r_ntc    = kNtcR0 * (v_cc - v_out) / v_out;
    const float t_kelvin = 1.0f / (1.0f / kNtcT0 + logf(r_ntc / kNtcR0) / kNtcBeta);
    temperature_c = t_kelvin - 273.15f;
    return ESP_OK;
}

esp_err_t SensorManager::read_moisture(int &raw, int &mv, float &pct) {
    if (g_adc1_handle == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(adc_oneshot_read(g_adc1_handle, kMoistureAdcChannel, &raw), TAG, "moisture ADC read failed");

    if (g_moisture_cali_ok && g_moisture_cali != nullptr) {
        ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(g_moisture_cali, raw, &mv), TAG, "moisture cali failed");
    } else {
        mv = (raw * 3300) / 4095;
    }

    // Higher voltage = drier, so invert for intuitive wet percentage
    pct = (1.0f - static_cast<float>(mv) / 3300.0f) * 100.0f;
    if (pct > 100.0f) pct = 100.0f;
    if (pct < 0.0f)   pct = 0.0f;

    return ESP_OK;
}

esp_err_t SensorManager::read_light(int &raw, int &mv, float &pct) {
    if (g_adc1_handle == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(adc_oneshot_read(g_adc1_handle, kLightAdcChannel, &raw), TAG, "light ADC read failed");

    if (g_light_cali_ok && g_light_cali != nullptr) {
        ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(g_light_cali, raw, &mv), TAG, "light cali failed");
    } else {
        mv = (raw * 3300) / 4095;
    }

    // Higher voltage = brighter (LDR resistance drops with light)
    pct = static_cast<float>(mv) / 3300.0f * 100.0f;
    if (pct > 100.0f) pct = 100.0f;
    if (pct < 0.0f)   pct = 0.0f;

    return ESP_OK;
}
