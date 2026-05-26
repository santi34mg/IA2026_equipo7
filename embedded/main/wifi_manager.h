#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

enum class WifiConnectResult {
    Pending,
    Ok,
    FailAuth,
    FailNotFound,
    FailOther,
    Timeout,
};

class WifiManager {
public:
    esp_err_t init();
    bool is_connected() const;

    WifiConnectResult try_connect_blocking(const char *ssid, const char *password, uint32_t timeout_ms);

    // Tries NVS credentials first (if present), then each entry of kKnownNetworks[]
    // in order. Returns Ok on the first success; otherwise returns the last failure.
    WifiConnectResult auto_connect_blocking(uint32_t per_attempt_timeout_ms);

    void get_ip_string(char *out, size_t out_len) const;
    void get_connected_ssid(char *out, size_t out_len) const;

    // Active scan — prints every visible AP to stdout with [scan] prefix.
    // Blocks until the scan completes (~2-3 seconds typical).
    esp_err_t scan_and_print();

    static esp_err_t save_credentials(const char *ssid, const char *password);
    static const char *result_to_string(WifiConnectResult r);
};

extern WifiManager g_wifi_manager;
