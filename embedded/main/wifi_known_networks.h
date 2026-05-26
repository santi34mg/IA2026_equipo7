#pragma once

#include <stddef.h>

struct WifiKnownNetwork {
    const char *ssid;
    const char *password;
};

// Edit this list to add fallback networks. Tried in order at boot, after NVS.
// If you don't want any fallback networks, leave the array empty:
//     constexpr WifiKnownNetwork kKnownNetworks[] = {};
constexpr WifiKnownNetwork kKnownNetworks[] = {
    {"UM_WiFi", "umontevideo"},
    {"°o°", "clk29qfj"},
    {"Ignacio", "8Y23N9MS"},
    {"Nico", "8Y23N9MS"},
};

constexpr size_t kKnownNetworksCount = sizeof(kKnownNetworks) / sizeof(kKnownNetworks[0]);
