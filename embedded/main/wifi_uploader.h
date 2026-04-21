#pragma once

#include <stdint.h>

#include "esp_err.h"

#include "sensor.h"

class WiFiUploader {
public:
    esp_err_t init();
    esp_err_t send_reading(int64_t timestamp_epoch, const SensorData &data);

private:
    bool configured() const;
    bool connected() const;
};
