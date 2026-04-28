#include "storage.h"

#include <inttypes.h>
#include <stdio.h>
#include <sys/stat.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_spiffs.h"

namespace {
constexpr const char *kBasePath = "/data";
constexpr const char *kCsvPath = "/data/log.csv";
const char *TAG = "storage";
}  // namespace

esp_err_t storage_csv::build_header(char *buffer, size_t buffer_len, size_t *written_len) {
    if (buffer == nullptr || written_len == nullptr || buffer_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const int len = snprintf(
        buffer,
        buffer_len,
        "timestamp_epoch,dht11_temp_c,dht11_humidity_pct,ks0033_temp_c,moisture_raw,light_raw\n"
    );
    if (len < 0 || static_cast<size_t>(len) >= buffer_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    *written_len = static_cast<size_t>(len);
    return ESP_OK;
}

esp_err_t storage_csv::build_row(
    char *buffer,
    size_t buffer_len,
    size_t *written_len,
    int64_t timestamp_epoch,
    const SensorData &data
) {
    if (buffer == nullptr || written_len == nullptr || buffer_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const int len = snprintf(
        buffer,
        buffer_len,
        "%" PRId64 ",%.2f,%.2f,%.2f,%d,%d\n",
        timestamp_epoch,
        static_cast<double>(data.dht11_temperature_c),
        static_cast<double>(data.dht11_humidity_pct),
        static_cast<double>(data.ks0033_temperature_c),
        data.moisture_raw,
        data.light_raw
    );
    if (len < 0 || static_cast<size_t>(len) >= buffer_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    *written_len = static_cast<size_t>(len);
    return ESP_OK;
}

esp_err_t StorageManager::init() {
    esp_vfs_spiffs_conf_t conf = {};
    conf.base_path = kBasePath;
    conf.partition_label = nullptr;
    conf.max_files = 5;
    conf.format_if_mount_failed = true;

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
        return ret;
    }

    size_t total = 0;
    size_t used = 0;
    ret = esp_spiffs_info(nullptr, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS mounted: total=%u used=%u", static_cast<unsigned>(total), static_cast<unsigned>(used));
    } else {
        ESP_LOGW(TAG, "Failed to query SPIFFS info: %s", esp_err_to_name(ret));
    }

    return ensure_csv_header();
}

esp_err_t StorageManager::ensure_csv_header() {
    struct stat st = {};
    if (stat(kCsvPath, &st) == 0) {
        return ESP_OK;
    }

    FILE *file = fopen(kCsvPath, "w");
    if (file == nullptr) {
        ESP_LOGE(TAG, "Cannot create CSV file at %s", kCsvPath);
        return ESP_FAIL;
    }

    char header[256] = {};
    size_t header_len = 0;
    esp_err_t fmt_ret = storage_csv::build_header(header, sizeof(header), &header_len);
    if (fmt_ret != ESP_OK) {
        fclose(file);
        ESP_LOGE(TAG, "Failed to format CSV header: %s", esp_err_to_name(fmt_ret));
        return fmt_ret;
    }

    const int written = fprintf(file, "%s", header);
    fclose(file);

    if (written <= 0) {
        ESP_LOGE(TAG, "Failed writing CSV header");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void StorageManager::dump_csv_to_serial() {
    FILE *file = fopen(kCsvPath, "r");
    if (file == nullptr) {
        ESP_LOGW(TAG, "No CSV file to dump");
        return;
    }

    printf("\n--- CSV DUMP START ---\n");
    char line[256] = {};
    while (fgets(line, sizeof(line), file) != nullptr) {
        printf("%s", line);
    }
    printf("--- CSV DUMP END ---\n\n");
    fclose(file);
}

esp_err_t StorageManager::append_row(int64_t timestamp_epoch, const SensorData &data) {
    FILE *file = fopen(kCsvPath, "a");
    if (file == nullptr) {
        ESP_LOGE(TAG, "Failed opening %s for append", kCsvPath);
        return ESP_FAIL;
    }

    char row[192] = {};
    size_t row_len = 0;
    esp_err_t fmt_ret = storage_csv::build_row(row, sizeof(row), &row_len, timestamp_epoch, data);
    if (fmt_ret != ESP_OK) {
        fclose(file);
        ESP_LOGE(TAG, "Failed to format CSV row: %s", esp_err_to_name(fmt_ret));
        return fmt_ret;
    }

    const int written = fprintf(file, "%s", row);

    fflush(file);
    fclose(file);

    if (written <= 0) {
        ESP_LOGE(TAG, "Failed to append row to CSV");
        return ESP_FAIL;
    }

    // Stream each new sample to serial as it's logged (row already ends in '\n')
    printf("%s", row);

    return ESP_OK;
}
