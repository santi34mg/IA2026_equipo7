#include "time_service.h"

#include <time.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
const char *TAG = "time_service";

bool is_time_valid() {
    time_t now = 0;
    time(&now);
    struct tm timeinfo = {};
    gmtime_r(&now, &timeinfo);
    return timeinfo.tm_year >= (2023 - 1900);
}
}  // namespace

esp_err_t TimeService::init_sntp() {
    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event loop create failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_sntp_stop();
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, const_cast<char *>("pool.ntp.org"));
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP initialized");
    return ESP_OK;
}

bool TimeService::wait_for_sync(uint32_t timeout_seconds) {
    for (uint32_t i = 0; i < timeout_seconds; ++i) {
        if (is_time_valid()) {
            ESP_LOGI(TAG, "Time synchronized");
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    return false;
}

int64_t TimeService::current_epoch_seconds() {
    time_t now = 0;
    time(&now);
    return static_cast<int64_t>(now);
}
