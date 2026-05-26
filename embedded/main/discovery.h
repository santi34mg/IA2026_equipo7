#pragma once

#include <stdint.h>

#include "esp_err.h"

// Starts a task that broadcasts "PLANT_MONITOR:<ip>:<tcp_port>\n" to
// 255.255.255.255:8081 every ~3 seconds while WiFi is connected.
// Idempotent — calling it again after success is a no-op.
esp_err_t discovery_start(uint16_t tcp_port);
