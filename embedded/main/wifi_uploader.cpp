#include "wifi_uploader.h"

#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

namespace {
#ifndef APP_WIFI_SSID
#define APP_WIFI_SSID ""
#endif

#ifndef APP_WIFI_PASSWORD
#define APP_WIFI_PASSWORD ""
#endif

#ifndef APP_UPLOAD_URL
#define APP_UPLOAD_URL ""
#endif

constexpr TickType_t kConnectWaitTicks = pdMS_TO_TICKS(15000);
constexpr int kWifiConnectedBit = BIT0;
constexpr int kWifiFailBit = BIT1;

const char *TAG = "wifi_uploader";
EventGroupHandle_t g_wifi_event_group = nullptr;
bool g_wifi_started = false;

void wifi_event_handler(void *, esp_event_base_t event_base, int32_t event_id, void *) {
    if (g_wifi_event_group == nullptr) {
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(g_wifi_event_group, kWifiConnectedBit);
        xEventGroupSetBits(g_wifi_event_group, kWifiFailBit);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupClearBits(g_wifi_event_group, kWifiFailBit);
        xEventGroupSetBits(g_wifi_event_group, kWifiConnectedBit);
    }
}
}  // namespace

bool WiFiUploader::configured() const {
    return strlen(APP_WIFI_SSID) > 0 && strlen(APP_UPLOAD_URL) > 0;
}

bool WiFiUploader::connected() const {
    if (g_wifi_event_group == nullptr) {
        return false;
    }

    EventBits_t bits = xEventGroupGetBits(g_wifi_event_group);
    return (bits & kWifiConnectedBit) != 0;
}

esp_err_t WiFiUploader::init() {
    if (!configured()) {
        ESP_LOGW(TAG, "Wi-Fi sender disabled: set APP_WIFI_SSID and APP_UPLOAD_URL build macros");
        return ESP_ERR_NOT_SUPPORTED;
    }

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

    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif == nullptr) {
        sta_netif = esp_netif_create_default_wifi_sta();
    }

    if (sta_netif == nullptr) {
        ESP_LOGE(TAG, "Failed to create default Wi-Fi station netif");
        return ESP_FAIL;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (g_wifi_event_group == nullptr) {
        g_wifi_event_group = xEventGroupCreate();
        if (g_wifi_event_group == nullptr) {
            ESP_LOGE(TAG, "Failed to create Wi-Fi event group");
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr, nullptr));

    wifi_config_t wifi_config = {};
    snprintf(reinterpret_cast<char *>(wifi_config.sta.ssid), sizeof(wifi_config.sta.ssid), "%s", APP_WIFI_SSID);
    snprintf(reinterpret_cast<char *>(wifi_config.sta.password), sizeof(wifi_config.sta.password), "%s", APP_WIFI_PASSWORD);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    if (!g_wifi_started) {
        ESP_ERROR_CHECK(esp_wifi_start());
        g_wifi_started = true;
    }

    EventBits_t bits = xEventGroupWaitBits(
        g_wifi_event_group,
        kWifiConnectedBit | kWifiFailBit,
        pdFALSE,
        pdFALSE,
        kConnectWaitTicks
    );

    if ((bits & kWifiConnectedBit) != 0) {
        ESP_LOGI(TAG, "Connected to Wi-Fi SSID '%s'", APP_WIFI_SSID);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Wi-Fi connect timeout/failure for SSID '%s'", APP_WIFI_SSID);
    return ESP_ERR_TIMEOUT;
}

esp_err_t WiFiUploader::send_reading(int64_t timestamp_epoch, const SensorData &data) {
    if (!configured()) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    char payload[256] = {};
    const int payload_len = snprintf(
        payload,
        sizeof(payload),
        "{\"timestamp_epoch\":%lld,\"temp_c\":%.2f,\"humidity_pct\":%.2f,\"pressure_pa\":%.2f,\"adc_raw\":%d,\"adc_mv\":%d,\"bme280_ok\":%d,\"adc_ok\":%d}",
        static_cast<long long>(timestamp_epoch),
        static_cast<double>(data.temperature_c),
        static_cast<double>(data.humidity_pct),
        static_cast<double>(data.pressure_pa),
        data.adc_raw,
        data.adc_mv,
        data.bme280_ok ? 1 : 0,
        data.adc_ok ? 1 : 0
    );

    if (payload_len <= 0 || payload_len >= static_cast<int>(sizeof(payload))) {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_http_client_config_t config = {};
    config.url = APP_UPLOAD_URL;
    config.method = HTTP_METHOD_POST;
    config.timeout_ms = 5000;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == nullptr) {
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, payload, payload_len);

    esp_err_t ret = esp_http_client_perform(client);
    if (ret == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        if (status < 200 || status >= 300) {
            ESP_LOGW(TAG, "Upload returned HTTP %d", status);
            ret = ESP_FAIL;
        }
    } else {
        ESP_LOGW(TAG, "Upload failed: %s", esp_err_to_name(ret));
    }

    esp_http_client_cleanup(client);
    return ret;
}
