#include "wifi_manager.h"

#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/ip_addr.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "wifi_known_networks.h"

namespace {
const char *TAG = "wifi_mgr";

constexpr const char *kNvsNamespace = "wifi_cfg";
constexpr const char *kNvsKeySsid = "ssid";
constexpr const char *kNvsKeyPass = "pass";

constexpr EventBits_t kConnectedBit = BIT0;
constexpr EventBits_t kFailBit = BIT1;
constexpr EventBits_t kStartedBit = BIT2;

bool s_stack_initialized = false;
bool s_connected = false;
bool s_attempt_in_flight = false;
bool s_tearing_down = false;
bool s_has_creds = false;
EventGroupHandle_t s_event_group = nullptr;
WifiConnectResult s_last_fail = WifiConnectResult::FailOther;
char s_ip_str[16] = "0.0.0.0";
char s_connected_ssid[33] = {};

WifiConnectResult map_disconnect_reason(uint8_t reason) {
    switch (reason) {
        case WIFI_REASON_NO_AP_FOUND:
            return WifiConnectResult::FailNotFound;
        case WIFI_REASON_AUTH_FAIL:
        case WIFI_REASON_AUTH_EXPIRE:
        case WIFI_REASON_HANDSHAKE_TIMEOUT:
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        case WIFI_REASON_ASSOC_NOT_AUTHED:
            return WifiConnectResult::FailAuth;
        default:
            return WifiConnectResult::FailOther;
    }
}

void event_handler(void * /*arg*/, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi station started");
        if (s_event_group != nullptr) {
            xEventGroupSetBits(s_event_group, kStartedBit);
        }
        return;
    }

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const auto *ev = static_cast<wifi_event_sta_disconnected_t *>(data);
        s_connected = false;
        if (s_tearing_down) {
            // We initiated this disconnect ourselves between attempts. Ignore.
            return;
        }
        if (s_attempt_in_flight) {
            s_last_fail = map_disconnect_reason(ev->reason);
            ESP_LOGW(TAG, "Connect attempt failed (reason=%u)", ev->reason);
            if (s_event_group != nullptr) {
                xEventGroupSetBits(s_event_group, kFailBit);
            }
        } else if (s_has_creds) {
            ESP_LOGW(TAG, "WiFi disconnected (reason=%u) — reconnecting", ev->reason);
            esp_wifi_connect();
        }
        return;
    }

    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const auto *ev = static_cast<ip_event_got_ip_t *>(data);
        esp_ip4addr_ntoa(&ev->ip_info.ip, s_ip_str, sizeof(s_ip_str));
        s_connected = true;
        ESP_LOGI(TAG, "Got IP: %s", s_ip_str);
        if (s_event_group != nullptr) {
            xEventGroupSetBits(s_event_group, kConnectedBit);
        }
        return;
    }
}

esp_err_t load_nvs_credentials(char *ssid_out, size_t ssid_len, char *pass_out, size_t pass_len, bool *has_creds) {
    *has_creds = false;
    ssid_out[0] = '\0';
    pass_out[0] = '\0';

    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    size_t len = ssid_len;
    err = nvs_get_str(handle, kNvsKeySsid, ssid_out, &len);
    if (err != ESP_OK) {
        nvs_close(handle);
        return (err == ESP_ERR_NVS_NOT_FOUND) ? ESP_OK : err;
    }

    len = pass_len;
    err = nvs_get_str(handle, kNvsKeyPass, pass_out, &len);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        return err;
    }

    nvs_close(handle);
    *has_creds = (ssid_out[0] != '\0');
    return ESP_OK;
}
}  // namespace

WifiManager g_wifi_manager;

esp_err_t WifiManager::init() {
    if (s_stack_initialized) {
        return ESP_OK;
    }

    s_event_group = xEventGroupCreate();
    if (s_event_group == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK) {
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, nullptr, nullptr);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, nullptr, nullptr);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        return err;
    }

    xEventGroupClearBits(s_event_group, kStartedBit | kConnectedBit | kFailBit);

    err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }

    // Wait for WIFI_EVENT_STA_START so subsequent esp_wifi_connect() calls are valid.
    xEventGroupWaitBits(s_event_group, kStartedBit, pdFALSE, pdFALSE, pdMS_TO_TICKS(5000));

    s_stack_initialized = true;
    return ESP_OK;
}

bool WifiManager::is_connected() const {
    return s_connected;
}

void WifiManager::get_ip_string(char *out, size_t out_len) const {
    if (out == nullptr || out_len == 0) {
        return;
    }
    strncpy(out, s_ip_str, out_len - 1);
    out[out_len - 1] = '\0';
}

void WifiManager::get_connected_ssid(char *out, size_t out_len) const {
    if (out == nullptr || out_len == 0) {
        return;
    }
    strncpy(out, s_connected_ssid, out_len - 1);
    out[out_len - 1] = '\0';
}

WifiConnectResult WifiManager::try_connect_blocking(const char *ssid, const char *password, uint32_t timeout_ms) {
    if (!s_stack_initialized || s_event_group == nullptr) {
        return WifiConnectResult::FailOther;
    }
    if (ssid == nullptr || ssid[0] == '\0') {
        return WifiConnectResult::FailOther;
    }

    // Tear down any previous association without letting the resulting
    // STA_DISCONNECTED event trip the FailBit or kick off a background reconnect.
    s_tearing_down = true;
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(300));
    s_tearing_down = false;

    // Now arm the new attempt — bits cleared AFTER the teardown settled.
    xEventGroupClearBits(s_event_group, kConnectedBit | kFailBit);
    s_last_fail = WifiConnectResult::FailOther;
    s_connected = false;
    s_has_creds = true;
    s_attempt_in_flight = true;

    wifi_config_t wifi_cfg = {};
    strncpy(reinterpret_cast<char *>(wifi_cfg.sta.ssid), ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    if (password != nullptr) {
        strncpy(reinterpret_cast<char *>(wifi_cfg.sta.password), password, sizeof(wifi_cfg.sta.password) - 1);
    }

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_config failed: %s", esp_err_to_name(err));
        s_attempt_in_flight = false;
        return WifiConnectResult::FailOther;
    }

    err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "connect failed: %s", esp_err_to_name(err));
        s_attempt_in_flight = false;
        return WifiConnectResult::FailOther;
    }

    const EventBits_t bits = xEventGroupWaitBits(
        s_event_group,
        kConnectedBit | kFailBit,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(timeout_ms)
    );

    s_attempt_in_flight = false;

    if (bits & kConnectedBit) {
        strncpy(s_connected_ssid, ssid, sizeof(s_connected_ssid) - 1);
        s_connected_ssid[sizeof(s_connected_ssid) - 1] = '\0';
        return WifiConnectResult::Ok;
    }
    if (bits & kFailBit) {
        return s_last_fail;
    }
    return WifiConnectResult::Timeout;
}

WifiConnectResult WifiManager::auto_connect_blocking(uint32_t per_attempt_timeout_ms) {
    WifiConnectResult last_result = WifiConnectResult::FailOther;

    char nvs_ssid[33] = {};
    char nvs_pass[65] = {};
    bool nvs_has_creds = false;
    const esp_err_t nvs_err = load_nvs_credentials(nvs_ssid, sizeof(nvs_ssid), nvs_pass, sizeof(nvs_pass), &nvs_has_creds);
    if (nvs_err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read NVS creds: %s", esp_err_to_name(nvs_err));
    }

    if (nvs_has_creds) {
        ESP_LOGI(TAG, "Trying NVS SSID \"%s\"", nvs_ssid);
        last_result = try_connect_blocking(nvs_ssid, nvs_pass, per_attempt_timeout_ms);
        if (last_result == WifiConnectResult::Ok) {
            return last_result;
        }
        ESP_LOGW(TAG, "NVS SSID failed (%s) — trying fallback list", result_to_string(last_result));
    } else {
        ESP_LOGI(TAG, "No NVS credentials — trying fallback list");
    }

    for (size_t i = 0; i < kKnownNetworksCount; ++i) {
        const auto &net = kKnownNetworks[i];
        if (net.ssid == nullptr || net.ssid[0] == '\0') {
            continue;
        }
        ESP_LOGI(TAG, "Trying known SSID [%u/%u] \"%s\"", static_cast<unsigned>(i + 1), static_cast<unsigned>(kKnownNetworksCount), net.ssid);
        last_result = try_connect_blocking(net.ssid, net.password ? net.password : "", per_attempt_timeout_ms);
        if (last_result == WifiConnectResult::Ok) {
            return last_result;
        }
        ESP_LOGW(TAG, "SSID \"%s\" failed (%s)", net.ssid, result_to_string(last_result));
    }

    s_has_creds = false;  // nothing worked — stop auto-reconnecting in handler
    return last_result;
}

esp_err_t WifiManager::save_credentials(const char *ssid, const char *password) {
    if (ssid == nullptr || password == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_str(handle, kNvsKeySsid, ssid);
    if (err != ESP_OK) {
        nvs_close(handle);
        return err;
    }
    err = nvs_set_str(handle, kNvsKeyPass, password);
    if (err != ESP_OK) {
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

const char *WifiManager::result_to_string(WifiConnectResult r) {
    switch (r) {
        case WifiConnectResult::Ok:           return "ok";
        case WifiConnectResult::FailAuth:     return "auth";
        case WifiConnectResult::FailNotFound: return "not_found";
        case WifiConnectResult::FailOther:    return "other";
        case WifiConnectResult::Timeout:      return "timeout";
        case WifiConnectResult::Pending:      return "pending";
    }
    return "other";
}
