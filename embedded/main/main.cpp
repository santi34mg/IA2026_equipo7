#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "sensor.h"
#include "storage.h"
#include "time_service.h"

namespace {
constexpr uint32_t kSamplePeriodMs = 2500;
constexpr uint32_t kLoggerStackSize = 6144;
constexpr UBaseType_t kLoggerTaskPriority = 5;

const char *TAG = "app_main";

SensorManager g_sensor_manager;
StorageManager g_storage_manager;

void logger_task(void * /*arg*/) {
    ESP_LOGI(TAG, "Logger task started (period: %" PRIu32 " ms)", kSamplePeriodMs);

    while (true) {
        SensorData sensor_data{};
        if (g_sensor_manager.read(sensor_data) != ESP_OK) {
            ESP_LOGW(TAG, "Sensor read completed with one or more errors");
        }

        const int64_t timestamp_epoch = TimeService::current_epoch_seconds();
        if (g_storage_manager.append_row(timestamp_epoch, sensor_data) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to append CSV row");
        }

        vTaskDelay(pdMS_TO_TICKS(kSamplePeriodMs));
    }
}
}  // namespace

extern "C" void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(g_sensor_manager.init());

    ret = g_storage_manager.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Storage init failed: %s", esp_err_to_name(ret));
        return;
    }

    // Print only the CSV header at boot so columns are labeled; each new row
    // is streamed live by append_row() as the sample period elapses.
    char header[256] = {};
    size_t header_len = 0;
    if (storage_csv::build_header(header, sizeof(header), &header_len) == ESP_OK) {
        printf("%s", header);
    }

    BaseType_t task_ok = xTaskCreate(
        logger_task,
        "logger_task",
        kLoggerStackSize,
        nullptr,
        kLoggerTaskPriority,
        nullptr
    );
    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create logger task");
    }
}
