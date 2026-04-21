#include "sensor.h"

#include <string.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

namespace {
constexpr gpio_num_t kI2cSdaPin = GPIO_NUM_21;
constexpr gpio_num_t kI2cSclPin = GPIO_NUM_22;
constexpr i2c_port_t kI2cPort = I2C_NUM_0;
constexpr uint32_t kI2cFreqHz = 100000;

constexpr uint8_t kBme280Addr = 0x76;
constexpr uint8_t kBme280RegChipId = 0xD0;
constexpr uint8_t kBme280ExpectedChipId = 0x60;
constexpr uint8_t kBme280RegCtrlHum = 0xF2;
constexpr uint8_t kBme280RegStatus = 0xF3;
constexpr uint8_t kBme280RegCtrlMeas = 0xF4;
constexpr uint8_t kBme280RegConfig = 0xF5;
constexpr uint8_t kBme280RegPressMsb = 0xF7;
constexpr uint8_t kBme280RegCalib00 = 0x88;
constexpr uint8_t kBme280RegCalib26 = 0xE1;

constexpr adc_unit_t kAdcUnit = ADC_UNIT_1;
constexpr adc_channel_t kAdcChannel = ADC_CHANNEL_6;  // GPIO34
constexpr adc_atten_t kAdcAtten = ADC_ATTEN_DB_12;

const char *TAG = "sensor";

struct Bme280CalibData {
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;

    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;

    uint8_t dig_h1;
    int16_t dig_h2;
    uint8_t dig_h3;
    int16_t dig_h4;
    int16_t dig_h5;
    int8_t dig_h6;
};

struct Bme280Context {
    Bme280CalibData calib{};
    int32_t t_fine = 0;
    bool initialized = false;
};

Bme280Context g_bme_ctx;
i2c_master_bus_handle_t g_i2c_bus = nullptr;
i2c_master_dev_handle_t g_bme_dev = nullptr;
adc_oneshot_unit_handle_t g_adc_handle = nullptr;
adc_cali_handle_t g_adc_cali = nullptr;
bool g_adc_cali_enabled = false;

esp_err_t i2c_write_reg(uint8_t reg, uint8_t value) {
    uint8_t payload[2] = {reg, value};
    if (g_bme_dev == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit(g_bme_dev, payload, sizeof(payload), 100);
}

esp_err_t i2c_read_regs(uint8_t start_reg, uint8_t *buffer, size_t len) {
    if (g_bme_dev == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit_receive(g_bme_dev, &start_reg, 1, buffer, len, 100);
}

uint16_t u16_le(const uint8_t *data) {
    return static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8));
}

int16_t s16_le(const uint8_t *data) {
    return static_cast<int16_t>(u16_le(data));
}
}  // namespace

esp_err_t SensorManager::init() {
    ESP_RETURN_ON_ERROR(init_i2c(), TAG, "I2C init failed");
    ESP_RETURN_ON_ERROR(init_bme280(), TAG, "BME280 init failed");
    ESP_RETURN_ON_ERROR(init_adc(), TAG, "ADC init failed");
    return ESP_OK;
}

esp_err_t SensorManager::read(SensorData &out_data) {
    memset(&out_data, 0, sizeof(out_data));

    esp_err_t result = ESP_OK;

    if (read_bme280(out_data.temperature_c, out_data.humidity_pct, out_data.pressure_pa) == ESP_OK) {
        out_data.bme280_ok = true;
    } else {
        out_data.bme280_ok = false;
        result = ESP_FAIL;
    }

    if (read_adc(out_data.adc_raw, out_data.adc_mv) == ESP_OK) {
        out_data.adc_ok = true;
    } else {
        out_data.adc_ok = false;
        result = ESP_FAIL;
    }

    return result;
}

esp_err_t SensorManager::init_i2c() {
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = kI2cPort;
    bus_config.sda_io_num = kI2cSdaPin;
    bus_config.scl_io_num = kI2cSclPin;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &g_i2c_bus), TAG, "i2c_new_master_bus failed");

    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = kBme280Addr;
    dev_config.scl_speed_hz = kI2cFreqHz;

    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(g_i2c_bus, &dev_config, &g_bme_dev), TAG, "i2c add device failed");

    return ESP_OK;
}

esp_err_t SensorManager::init_bme280() {
    uint8_t chip_id = 0;
    ESP_RETURN_ON_ERROR(i2c_read_regs(kBme280RegChipId, &chip_id, 1), TAG, "failed to read BME280 chip id");
    if (chip_id != kBme280ExpectedChipId) {
        ESP_LOGE(TAG, "unexpected BME280 chip id: 0x%02X", chip_id);
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t calib1[26] = {0};
    uint8_t calib2[7] = {0};
    ESP_RETURN_ON_ERROR(i2c_read_regs(kBme280RegCalib00, calib1, sizeof(calib1)), TAG, "failed calib1 read");
    ESP_RETURN_ON_ERROR(i2c_read_regs(kBme280RegCalib26, calib2, sizeof(calib2)), TAG, "failed calib2 read");

    g_bme_ctx.calib.dig_t1 = u16_le(&calib1[0]);
    g_bme_ctx.calib.dig_t2 = s16_le(&calib1[2]);
    g_bme_ctx.calib.dig_t3 = s16_le(&calib1[4]);

    g_bme_ctx.calib.dig_p1 = u16_le(&calib1[6]);
    g_bme_ctx.calib.dig_p2 = s16_le(&calib1[8]);
    g_bme_ctx.calib.dig_p3 = s16_le(&calib1[10]);
    g_bme_ctx.calib.dig_p4 = s16_le(&calib1[12]);
    g_bme_ctx.calib.dig_p5 = s16_le(&calib1[14]);
    g_bme_ctx.calib.dig_p6 = s16_le(&calib1[16]);
    g_bme_ctx.calib.dig_p7 = s16_le(&calib1[18]);
    g_bme_ctx.calib.dig_p8 = s16_le(&calib1[20]);
    g_bme_ctx.calib.dig_p9 = s16_le(&calib1[22]);

    g_bme_ctx.calib.dig_h1 = calib1[25];
    g_bme_ctx.calib.dig_h2 = s16_le(&calib2[0]);
    g_bme_ctx.calib.dig_h3 = calib2[2];
    g_bme_ctx.calib.dig_h4 = static_cast<int16_t>((static_cast<int16_t>(calib2[3]) << 4) | (calib2[4] & 0x0F));
    g_bme_ctx.calib.dig_h5 = static_cast<int16_t>((static_cast<int16_t>(calib2[5]) << 4) | ((calib2[4] >> 4) & 0x0F));
    g_bme_ctx.calib.dig_h6 = static_cast<int8_t>(calib2[6]);

    ESP_RETURN_ON_ERROR(i2c_write_reg(kBme280RegCtrlHum, 0x01), TAG, "write ctrl_hum failed");
    ESP_RETURN_ON_ERROR(i2c_write_reg(kBme280RegCtrlMeas, 0x27), TAG, "write ctrl_meas failed");
    ESP_RETURN_ON_ERROR(i2c_write_reg(kBme280RegConfig, 0xA0), TAG, "write config failed");

    g_bme_ctx.initialized = true;
    return ESP_OK;
}

esp_err_t SensorManager::init_adc() {
    adc_oneshot_unit_init_cfg_t init_config = {};
    init_config.unit_id = kAdcUnit;
    init_config.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
    init_config.ulp_mode = ADC_ULP_MODE_DISABLE;
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_config, &g_adc_handle), TAG, "adc_oneshot_new_unit failed");

    adc_oneshot_chan_cfg_t chan_config = {};
    chan_config.atten = kAdcAtten;
    chan_config.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(g_adc_handle, kAdcChannel, &chan_config), TAG, "adc channel config failed");

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {};
    cali_config.unit_id = kAdcUnit;
    cali_config.chan = kAdcChannel;
    cali_config.atten = kAdcAtten;
    cali_config.bitwidth = ADC_BITWIDTH_DEFAULT;
    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, &g_adc_cali);
    if (ret == ESP_OK) {
        g_adc_cali_enabled = true;
        ESP_LOGI(TAG, "ADC calibration enabled");
    } else {
        g_adc_cali_enabled = false;
        ESP_LOGW(TAG, "ADC calibration unavailable: %s", esp_err_to_name(ret));
    }
#endif

    return ESP_OK;
}

esp_err_t SensorManager::read_bme280(float &temperature_c, float &humidity_pct, float &pressure_pa) {
    if (!g_bme_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t status = 0;
    ESP_RETURN_ON_ERROR(i2c_read_regs(kBme280RegStatus, &status, 1), TAG, "status read failed");
    if (status & 0x08) {
        ESP_LOGW(TAG, "BME280 measurement still running");
    }

    uint8_t raw[8] = {0};
    ESP_RETURN_ON_ERROR(i2c_read_regs(kBme280RegPressMsb, raw, sizeof(raw)), TAG, "raw data read failed");

    const int32_t adc_p = (static_cast<int32_t>(raw[0]) << 12) |
                          (static_cast<int32_t>(raw[1]) << 4) |
                          (static_cast<int32_t>(raw[2]) >> 4);
    const int32_t adc_t = (static_cast<int32_t>(raw[3]) << 12) |
                          (static_cast<int32_t>(raw[4]) << 4) |
                          (static_cast<int32_t>(raw[5]) >> 4);
    const int32_t adc_h = (static_cast<int32_t>(raw[6]) << 8) |
                          static_cast<int32_t>(raw[7]);

    int32_t var1 = ((((adc_t >> 3) - (static_cast<int32_t>(g_bme_ctx.calib.dig_t1) << 1))) *
                    static_cast<int32_t>(g_bme_ctx.calib.dig_t2)) >>
                   11;
    int32_t var2 = (((((adc_t >> 4) - static_cast<int32_t>(g_bme_ctx.calib.dig_t1)) *
                      ((adc_t >> 4) - static_cast<int32_t>(g_bme_ctx.calib.dig_t1))) >>
                     12) *
                    static_cast<int32_t>(g_bme_ctx.calib.dig_t3)) >>
                   14;
    g_bme_ctx.t_fine = var1 + var2;
    const int32_t temp_x100 = (g_bme_ctx.t_fine * 5 + 128) >> 8;
    temperature_c = static_cast<float>(temp_x100) / 100.0F;

    int64_t pvar1 = static_cast<int64_t>(g_bme_ctx.t_fine) - 128000;
    int64_t pvar2 = pvar1 * pvar1 * static_cast<int64_t>(g_bme_ctx.calib.dig_p6);
    pvar2 = pvar2 + ((pvar1 * static_cast<int64_t>(g_bme_ctx.calib.dig_p5)) << 17);
    pvar2 = pvar2 + (static_cast<int64_t>(g_bme_ctx.calib.dig_p4) << 35);
    pvar1 = ((pvar1 * pvar1 * static_cast<int64_t>(g_bme_ctx.calib.dig_p3)) >> 8) +
            ((pvar1 * static_cast<int64_t>(g_bme_ctx.calib.dig_p2)) << 12);
    pvar1 = ((((static_cast<int64_t>(1) << 47) + pvar1) * static_cast<int64_t>(g_bme_ctx.calib.dig_p1)) >> 33);

    if (pvar1 == 0) {
        ESP_LOGE(TAG, "BME280 pressure compensation divide-by-zero");
        return ESP_FAIL;
    }

    int64_t pressure = 1048576 - adc_p;
    pressure = (((pressure << 31) - pvar2) * 3125) / pvar1;
    pvar1 = (static_cast<int64_t>(g_bme_ctx.calib.dig_p9) * (pressure >> 13) * (pressure >> 13)) >> 25;
    pvar2 = (static_cast<int64_t>(g_bme_ctx.calib.dig_p8) * pressure) >> 19;
    pressure = ((pressure + pvar1 + pvar2) >> 8) + (static_cast<int64_t>(g_bme_ctx.calib.dig_p7) << 4);
    pressure_pa = static_cast<float>(pressure) / 256.0F;

    int32_t h = g_bme_ctx.t_fine - 76800;
    h = (((((adc_h << 14) - (static_cast<int32_t>(g_bme_ctx.calib.dig_h4) << 20) -
            (static_cast<int32_t>(g_bme_ctx.calib.dig_h5) * h)) +
           16384) >>
          15) *
         (((((((h * static_cast<int32_t>(g_bme_ctx.calib.dig_h6)) >> 10) *
              (((h * static_cast<int32_t>(g_bme_ctx.calib.dig_h3)) >> 11) + 32768)) >>
             10) +
            2097152) *
               static_cast<int32_t>(g_bme_ctx.calib.dig_h2) +
           8192) >>
          14));
    h = h - (((((h >> 15) * (h >> 15)) >> 7) * static_cast<int32_t>(g_bme_ctx.calib.dig_h1)) >> 4);
    h = (h < 0 ? 0 : h);
    h = (h > 419430400 ? 419430400 : h);
    humidity_pct = static_cast<float>(h >> 12) / 1024.0F;

    return ESP_OK;
}

esp_err_t SensorManager::read_adc(int &raw, int &mv) {
    if (g_adc_handle == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(adc_oneshot_read(g_adc_handle, kAdcChannel, &raw), TAG, "adc read failed");

    if (g_adc_cali_enabled && g_adc_cali != nullptr) {
        ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(g_adc_cali, raw, &mv), TAG, "adc calibration failed");
    } else {
        mv = 0;
    }

    return ESP_OK;
}
