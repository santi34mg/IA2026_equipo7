#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t tcp_server_start(uint16_t port);
void tcp_server_broadcast(const char *data, size_t len);
