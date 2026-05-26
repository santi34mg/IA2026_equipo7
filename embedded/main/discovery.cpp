#include "discovery.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wifi_manager.h"

namespace {
const char *TAG = "discovery";

constexpr uint16_t kDiscoveryPort = 8081;
constexpr uint32_t kBroadcastIntervalMs = 3000;
constexpr uint32_t kTaskStackSize = 3072;
constexpr UBaseType_t kTaskPriority = 3;

bool s_task_started = false;
uint16_t s_tcp_port = 0;
int s_sock = -1;

int create_broadcast_socket() {
    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
        return -1;
    }
    int opt = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt)) < 0) {
        ESP_LOGE(TAG, "SO_BROADCAST failed: errno=%d", errno);
        close(fd);
        return -1;
    }
    return fd;
}

void discovery_task(void * /*arg*/) {
    sockaddr_in target = {};
    target.sin_family = AF_INET;
    target.sin_port = htons(kDiscoveryPort);
    target.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    int fail_streak = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(kBroadcastIntervalMs));

        if (!g_wifi_manager.is_connected()) {
            continue;
        }

        if (s_sock < 0) {
            s_sock = create_broadcast_socket();
            if (s_sock < 0) {
                continue;
            }
        }

        char ip[16] = {};
        g_wifi_manager.get_ip_string(ip, sizeof(ip));

        char packet[64] = {};
        const int len = snprintf(packet, sizeof(packet), "PLANT_MONITOR:%s:%u\n", ip, static_cast<unsigned>(s_tcp_port));
        if (len <= 0) {
            continue;
        }

        const int sent = sendto(s_sock, packet, static_cast<size_t>(len), 0, reinterpret_cast<sockaddr *>(&target), sizeof(target));
        if (sent < 0) {
            ++fail_streak;
            ESP_LOGW(TAG, "sendto failed (errno=%d, streak=%d)", errno, fail_streak);
            if (fail_streak >= 3) {
                close(s_sock);
                s_sock = -1;
                fail_streak = 0;
            }
        } else {
            fail_streak = 0;
        }
    }
}
}  // namespace

esp_err_t discovery_start(uint16_t tcp_port) {
    if (s_task_started) {
        s_tcp_port = tcp_port;
        return ESP_OK;
    }

    s_tcp_port = tcp_port;
    const BaseType_t ok = xTaskCreate(discovery_task, "discovery", kTaskStackSize, nullptr, kTaskPriority, nullptr);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create discovery task");
        return ESP_ERR_NO_MEM;
    }
    s_task_started = true;
    ESP_LOGI(TAG, "Discovery broadcasting on UDP %u every %u ms", kDiscoveryPort, static_cast<unsigned>(kBroadcastIntervalMs));
    return ESP_OK;
}
