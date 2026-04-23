#pragma once

#include <stdint.h>

#include "esp_err.h"

struct SensorData {
    bool  dht11_ok;
    float dht11_temperature_c;
    float dht11_humidity_pct;

    bool  ks0033_ok;
    float ks0033_temperature_c;

    bool  moisture_ok;
    int   moisture_raw;
    int   moisture_mv;
    float moisture_pct;  // 0 = dry, 100 = fully wet

    bool  light_ok;
    int   light_raw;
    int   light_mv;
    float light_pct;  // 0 = dark, 100 = maximum light
};

class SensorManager {
public:
    esp_err_t init();
    esp_err_t read(SensorData &out_data);

private:
    esp_err_t init_dht11();
    esp_err_t init_ks0033();
    esp_err_t init_moisture();
    esp_err_t init_light();

    esp_err_t read_dht11(float &temperature_c, float &humidity_pct);
    esp_err_t read_ks0033(float &temperature_c);
    esp_err_t read_moisture(int &raw, int &mv, float &pct);
    esp_err_t read_light(int &raw, int &mv, float &pct);
};
