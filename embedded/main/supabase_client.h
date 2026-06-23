#pragma once

#include <stdint.h>

#include "esp_err.h"

#include "classifier.h"
#include "sensor.h"

// Cloud telemetry: ships each sensor reading to Supabase over HTTPS.
//
// This runs alongside the existing LAN TCP server / local CSV (dual-write):
// it never blocks the sensor loop. `supabase_client_enqueue()` drops the
// reading into a small queue and returns immediately; a dedicated background
// task performs the actual POST (with retry) only when WiFi is connected.

// Create the telemetry queue and start the background worker task.
// Safe to call once at boot, before WiFi is up — the worker simply waits.
esp_err_t supabase_client_start();

// Non-blocking. Queue one reading for upload. If the queue is full the oldest
// pending reading is dropped so the newest is always kept. Safe to call from
// the sensor/logger task.
void supabase_client_enqueue(int64_t timestamp_epoch, const SensorData &data,
                             PlantState predicted);
