#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "sensor.h"

namespace storage_csv {
esp_err_t build_header(char *buffer, size_t buffer_len, size_t *written_len);
esp_err_t build_row(char *buffer, size_t buffer_len, size_t *written_len, int64_t timestamp_epoch, const SensorData &data);
}  // namespace storage_csv

class StorageManager {
public:
    esp_err_t init();
    esp_err_t append_row(int64_t timestamp_epoch, const SensorData &data);

private:
    esp_err_t ensure_csv_header();
};
