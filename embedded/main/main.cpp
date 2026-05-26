#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "classifier.h"
#include "discovery.h"
#include "led_indicator.h"
#include "sensor.h"
#include "storage.h"
#include "tcp_server.h"
#include "time_service.h"
#include "wifi_manager.h"

namespace {
constexpr uint32_t kSamplePeriodMs = 2500;
constexpr uint32_t kLoggerStackSize = 6144;
constexpr UBaseType_t kLoggerTaskPriority = 5;

constexpr uint32_t kSerialCfgStackSize = 4096;
constexpr UBaseType_t kSerialCfgTaskPriority = 3;
constexpr uint16_t kTcpPort = 8080;
constexpr uint32_t kConnectTimeoutMs = 20000;

const char *TAG = "app_main";

SensorManager g_sensor_manager;
StorageManager g_storage_manager;
// g_wifi_manager is defined in wifi_manager.cpp (extern in wifi_manager.h)

void logger_task(void * /*arg*/) {
    ESP_LOGI(TAG, "Logger task started (period: %" PRIu32 " ms)", kSamplePeriodMs);

    while (true) {
        SensorData sensor_data{};
        if (g_sensor_manager.read(sensor_data) != ESP_OK) {
            ESP_LOGW(TAG, "Sensor read completed with one or more errors");
        }

        const PlantState predicted = classifier::predict(sensor_data);
        led_indicator::set_state(predicted);

        const int64_t timestamp_epoch = TimeService::current_epoch_seconds();
        if (g_storage_manager.append_row(timestamp_epoch, sensor_data, predicted) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to append CSV row");
        }

        vTaskDelay(pdMS_TO_TICKS(kSamplePeriodMs));
    }
}

void handle_wifi_command(char *line) {
    char *body = line + 5;  // skip "WIFI:"
    char *sep = strchr(body, ':');
    if (sep == nullptr) {
        printf("[wifi_cfg] PARSE_ERROR\n");
        return;
    }
    *sep = '\0';
    const char *ssid = body;
    const char *password = sep + 1;

    if (ssid[0] == '\0') {
        printf("[wifi_cfg] PARSE_ERROR\n");
        return;
    }

    esp_err_t err = WifiManager::save_credentials(ssid, password);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save credentials: %s", esp_err_to_name(err));
        printf("[wifi_cfg] CONNECT_FAIL reason=other\n");
        return;
    }

    const WifiConnectResult r = g_wifi_manager.try_connect_blocking(ssid, password, kConnectTimeoutMs);
    if (r == WifiConnectResult::Ok) {
        char ip_buf[16] = {};
        char ssid_buf[33] = {};
        g_wifi_manager.get_ip_string(ip_buf, sizeof(ip_buf));
        g_wifi_manager.get_connected_ssid(ssid_buf, sizeof(ssid_buf));
        printf("[wifi_cfg] CONNECT_OK ssid=%s ip=%s\n", ssid_buf, ip_buf);

        // Start (or re-confirm) the LAN services now that we're online.
        tcp_server_start(kTcpPort);
        discovery_start(kTcpPort);
    } else {
        printf("[wifi_cfg] CONNECT_FAIL reason=%s\n", WifiManager::result_to_string(r));
    }
}

void install_uart_console_driver() {
    constexpr int kRxBufSize = 256;
    const esp_err_t err = uart_driver_install(UART_NUM_0, kRxBufSize, 0, 0, nullptr, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return;
    }
    uart_vfs_dev_use_driver(UART_NUM_0);
    uart_vfs_dev_port_set_rx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CR);
}

void serial_config_task(void * /*arg*/) {
    ESP_LOGI(TAG, "Serial config task started — send WIFI:ssid:password to set credentials");

    char line[256];
    while (true) {
        if (fgets(line, sizeof(line), stdin) == nullptr) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        if (len == 0) {
            continue;
        }

        if (strncmp(line, "WIFI:", 5) == 0) {
            handle_wifi_command(line);
        }
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

    install_uart_console_driver();

    ESP_ERROR_CHECK(g_sensor_manager.init());
    led_indicator::init();

    ret = g_storage_manager.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Storage init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = g_wifi_manager.init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi init failed: %s — continuing with serial only", esp_err_to_name(ret));
    } else {
        const WifiConnectResult r = g_wifi_manager.auto_connect_blocking(kConnectTimeoutMs);
        if (r == WifiConnectResult::Ok) {
            char ssid[33] = {};
            char ip[16] = {};
            g_wifi_manager.get_connected_ssid(ssid, sizeof(ssid));
            g_wifi_manager.get_ip_string(ip, sizeof(ip));
            ESP_LOGI(TAG, "Online: ssid=\"%s\" ip=%s", ssid, ip);

            ret = tcp_server_start(kTcpPort);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "TCP server failed to start: %s", esp_err_to_name(ret));
            }
            ret = discovery_start(kTcpPort);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Discovery failed to start: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGW(TAG, "Boot-time connection failed (%s) — waiting for serial config",
                     WifiManager::result_to_string(r));
        }
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

    task_ok = xTaskCreate(
        serial_config_task,
        "serial_cfg",
        kSerialCfgStackSize,
        nullptr,
        kSerialCfgTaskPriority,
        nullptr
    );
    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create serial config task");
    }
}
