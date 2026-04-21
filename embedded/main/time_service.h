#pragma once

#include <stdint.h>

#include "esp_err.h"

class TimeService {
public:
    static esp_err_t init_sntp();
    static bool wait_for_sync(uint32_t timeout_seconds);
    static int64_t current_epoch_seconds();
};
