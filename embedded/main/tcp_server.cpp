#include "tcp_server.h"

#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace {
const char *TAG = "tcp_srv";

constexpr uint32_t kAcceptTaskStack = 4096;
constexpr UBaseType_t kAcceptTaskPriority = 4;

int s_listen_fd = -1;
int s_client_fd = -1;
SemaphoreHandle_t s_client_mutex = nullptr;
uint16_t s_port = 0;

void close_client_locked() {
    if (s_client_fd >= 0) {
        shutdown(s_client_fd, SHUT_RDWR);
        close(s_client_fd);
        s_client_fd = -1;
    }
}

void accept_task(void * /*arg*/) {
    while (true) {
        sockaddr_in client_addr = {};
        socklen_t addr_len = sizeof(client_addr);
        const int new_fd = accept(s_listen_fd, reinterpret_cast<sockaddr *>(&client_addr), &addr_len);
        if (new_fd < 0) {
            ESP_LOGW(TAG, "accept() failed: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        char ip_buf[16] = {};
        inet_ntoa_r(client_addr.sin_addr, ip_buf, sizeof(ip_buf));
        ESP_LOGI(TAG, "Client connected: %s", ip_buf);

        if (xSemaphoreTake(s_client_mutex, portMAX_DELAY) == pdTRUE) {
            close_client_locked();
            s_client_fd = new_fd;
            xSemaphoreGive(s_client_mutex);
        } else {
            close(new_fd);
        }
    }
}
}  // namespace

esp_err_t tcp_server_start(uint16_t port) {
    if (s_listen_fd >= 0) {
        return ESP_OK;
    }

    s_client_mutex = xSemaphoreCreateMutex();
    if (s_client_mutex == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    s_listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (s_listen_fd < 0) {
        ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
        return ESP_FAIL;
    }

    int opt = 1;
    setsockopt(s_listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(s_listen_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed: errno=%d", errno);
        close(s_listen_fd);
        s_listen_fd = -1;
        return ESP_FAIL;
    }

    if (listen(s_listen_fd, 1) < 0) {
        ESP_LOGE(TAG, "listen() failed: errno=%d", errno);
        close(s_listen_fd);
        s_listen_fd = -1;
        return ESP_FAIL;
    }

    s_port = port;
    ESP_LOGI(TAG, "TCP server listening on port %u", port);

    const BaseType_t ok = xTaskCreate(accept_task, "tcp_accept", kAcceptTaskStack, nullptr, kAcceptTaskPriority, nullptr);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create accept task");
        close(s_listen_fd);
        s_listen_fd = -1;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void tcp_server_broadcast(const char *data, size_t len) {
    if (data == nullptr || len == 0 || s_client_mutex == nullptr) {
        return;
    }

    if (xSemaphoreTake(s_client_mutex, 0) != pdTRUE) {
        return;
    }

    if (s_client_fd >= 0) {
        const int sent = send(s_client_fd, data, len, MSG_DONTWAIT);
        if (sent < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                ESP_LOGW(TAG, "Client send error (errno=%d) — closing", errno);
                close_client_locked();
            }
        }
    }

    xSemaphoreGive(s_client_mutex);
}
