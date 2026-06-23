#include "supabase_client.h"

#include <math.h>
#include <string.h>

#include "esp_http_client.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "supabase_config.h"
#include "wifi_manager.h"

namespace {
const char *TAG = "supabase";

// PostgREST insert endpoint for the readings table.
constexpr const char *kReadingsPath = "/rest/v1/sensor_readings";

// Background worker tuning.
constexpr UBaseType_t kQueueDepth = 8;
constexpr uint32_t kTaskStackSize = 6144;
constexpr UBaseType_t kTaskPriority = 4;
constexpr int kMaxAttempts = 3;
constexpr uint32_t kHttpTimeoutMs = 8000;

// One pending reading. We keep the raw inputs (not the formatted CSV row) so
// the JSON body matches the Supabase column names exactly.
struct PendingReading {
    int64_t ts_epoch;
    SensorData data;
    PlantState predicted;
};

QueueHandle_t s_queue = nullptr;
bool s_started = false;

// Render one numeric field as a JSON token, falling back to `null` when the
// sensor reported an error or produced a non-finite value (NaN/Inf would be
// invalid JSON and PostgREST would reject the whole row).
void float_token(char *out, size_t out_len, bool ok, float value) {
    if (ok && isfinite(value)) {
        snprintf(out, out_len, "%.2f", static_cast<double>(value));
    } else {
        strncpy(out, "null", out_len - 1);
        out[out_len - 1] = '\0';
    }
}

void int_token(char *out, size_t out_len, bool ok, int value) {
    if (ok) {
        snprintf(out, out_len, "%d", value);
    } else {
        strncpy(out, "null", out_len - 1);
        out[out_len - 1] = '\0';
    }
}

// Build the JSON insert body. Maps the firmware reading onto the existing
// `sensor_readings` columns: temperatura/luz/humedad_suelo/label.
// created_at is left to the server default (now()).
esp_err_t build_body(char *buf, size_t buf_len, const PendingReading &r) {
    char temp_tok[24];
    char luz_tok[16];
    char hum_tok[16];
    // temperatura = KS0033 (sensor que funciona); el DHT11 está roto.
    float_token(temp_tok, sizeof(temp_tok), r.data.ks0033_ok, r.data.ks0033_temperature_c);
    int_token(luz_tok, sizeof(luz_tok), r.data.light_ok, r.data.light_raw);
    int_token(hum_tok, sizeof(hum_tok), r.data.moisture_ok, r.data.moisture_raw);

    const int len = snprintf(
        buf, buf_len,
        "{\"temperatura\":%s,\"luz\":%s,\"humedad_suelo\":%s,\"label\":\"%s\"}",
        temp_tok, luz_tok, hum_tok, classifier::to_string(r.predicted));
    if (len < 0 || static_cast<size_t>(len) >= buf_len) {
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

// Perform a single POST. Returns ESP_OK only on a 2xx response.
esp_err_t post_once(esp_http_client_handle_t client, const char *body) {
    esp_http_client_set_post_field(client, body, strlen(body));
    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "POST transport error: %s", esp_err_to_name(err));
        return err;
    }
    const int status = esp_http_client_get_status_code(client);
    if (status >= 200 && status < 300) {
        ESP_LOGI(TAG, "POST ok (%d)", status);
        return ESP_OK;
    }
    ESP_LOGW(TAG, "POST rejected (HTTP %d)", status);
    return ESP_FAIL;
}

void telemetry_task(void * /*arg*/) {
    char url[256];
    snprintf(url, sizeof(url), "%s%s", SUPABASE_URL, kReadingsPath);

    esp_http_client_config_t cfg = {};
    cfg.url = url;
    cfg.method = HTTP_METHOD_POST;
    cfg.timeout_ms = kHttpTimeoutMs;
    // TLS server-cert verification is skipped (uni project) via
    // CONFIG_ESP_TLS_INSECURE + CONFIG_ESP_TLS_SKIP_SERVER_CERT_VERIFY in
    // sdkconfig, with no CA bundle attached. We deliberately do NOT set
    // skip_cert_common_name_check: that flag also disables SNI, and Supabase's
    // CDN needs SNI to present a certificate at all (without it the server
    // aborts the handshake with a fatal alert).
    cfg.keep_alive_enable = true;                   // reuse the TLS session

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == nullptr) {
        ESP_LOGE(TAG, "Failed to init HTTP client — telemetry disabled");
        vTaskDelete(nullptr);
        return;
    }

    esp_http_client_set_header(client, "apikey", SUPABASE_ANON_KEY);
    esp_http_client_set_header(client, "Authorization", "Bearer " SUPABASE_ANON_KEY);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Prefer", "return=minimal");

    ESP_LOGI(TAG, "Telemetry worker started -> %s", url);

    PendingReading r{};
    while (true) {
        if (xQueueReceive(s_queue, &r, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (!g_wifi_manager.is_connected()) {
            // Offline: the local CSV/SPIFFS already holds this row. Drop the
            // cloud copy rather than blocking. (Offline buffering is a v2 item.)
            ESP_LOGD(TAG, "Offline — skipping upload");
            continue;
        }

        char body[192];
        if (build_body(body, sizeof(body), r) != ESP_OK) {
            ESP_LOGW(TAG, "Body too large — skipping reading");
            continue;
        }

        uint32_t backoff_ms = 500;
        for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
            if (!g_wifi_manager.is_connected()) {
                break;
            }
            if (post_once(client, body) == ESP_OK) {
                break;
            }
            if (attempt < kMaxAttempts) {
                vTaskDelay(pdMS_TO_TICKS(backoff_ms));
                backoff_ms *= 3;  // 500ms -> 1500ms
            } else {
                ESP_LOGW(TAG, "Giving up on reading after %d attempts", kMaxAttempts);
            }
        }
    }
}
}  // namespace

esp_err_t supabase_client_start() {
    if (s_started) {
        return ESP_OK;
    }

    s_queue = xQueueCreate(kQueueDepth, sizeof(PendingReading));
    if (s_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create telemetry queue");
        return ESP_ERR_NO_MEM;
    }

    const BaseType_t ok = xTaskCreate(telemetry_task, "supabase_tx", kTaskStackSize,
                                      nullptr, kTaskPriority, nullptr);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create telemetry task");
        vQueueDelete(s_queue);
        s_queue = nullptr;
        return ESP_FAIL;
    }

    s_started = true;
    return ESP_OK;
}

void supabase_client_enqueue(int64_t timestamp_epoch, const SensorData &data,
                             PlantState predicted) {
    if (s_queue == nullptr) {
        return;
    }

    PendingReading r{};
    r.ts_epoch = timestamp_epoch;
    r.data = data;
    r.predicted = predicted;

    // Non-blocking. If full, drop the oldest so the freshest reading wins.
    if (xQueueSend(s_queue, &r, 0) != pdTRUE) {
        PendingReading discarded{};
        xQueueReceive(s_queue, &discarded, 0);
        xQueueSend(s_queue, &r, 0);
    }
}
