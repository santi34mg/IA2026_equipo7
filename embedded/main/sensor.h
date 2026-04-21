#pragma once

#include <stdint.h>

#include "esp_err.h"

struct SensorData {
    bool bme280_ok;
    float temperature_c;
    float humidity_pct;
    float pressure_pa;

    bool adc_ok;
    int adc_raw;
    int adc_mv;
};

class SensorManager {
public:
    esp_err_t init();
    esp_err_t read(SensorData &out_data);

private:
    esp_err_t init_i2c();
    esp_err_t init_bme280();
    esp_err_t init_adc();

    esp_err_t read_bme280(float &temperature_c, float &humidity_pct, float &pressure_pa);
    esp_err_t read_adc(int &raw, int &mv);
};
