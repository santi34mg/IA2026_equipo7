#include <string.h>

#include "unity.h"

#include "storage.h"

TEST_CASE("storage csv header format is stable", "[storage]") {
    char buffer[128] = {};
    size_t written_len = 0;

    const esp_err_t ret = storage_csv::build_header(buffer, sizeof(buffer), &written_len);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    const char *expected = "timestamp_epoch,temp_c,humidity_pct,pressure_pa,adc_raw,adc_mv,bme280_ok,adc_ok\n";
    TEST_ASSERT_EQUAL_STRING(expected, buffer);
    TEST_ASSERT_EQUAL(strlen(expected), written_len);
}

TEST_CASE("storage csv row format is stable", "[storage]") {
    SensorData data = {};
    data.bme280_ok = true;
    data.temperature_c = 23.456F;
    data.humidity_pct = 55.0F;
    data.pressure_pa = 101325.12F;
    data.adc_ok = false;
    data.adc_raw = 2048;
    data.adc_mv = 1100;

    char buffer[160] = {};
    size_t written_len = 0;

    const esp_err_t ret = storage_csv::build_row(buffer, sizeof(buffer), &written_len, 1713696000, data);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    const char *expected = "1713696000,23.46,55.00,101325.12,2048,1100,1,0\n";
    TEST_ASSERT_EQUAL_STRING(expected, buffer);
    TEST_ASSERT_EQUAL(strlen(expected), written_len);
}

TEST_CASE("storage csv formatting validates arguments", "[storage]") {
    size_t written_len = 0;
    SensorData data = {};

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, storage_csv::build_header(nullptr, 10, &written_len));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, storage_csv::build_row(nullptr, 10, &written_len, 1, data));
}

TEST_CASE("storage csv formatting detects short buffers", "[storage]") {
    char tiny[8] = {};
    size_t written_len = 0;
    SensorData data = {};

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, storage_csv::build_header(tiny, sizeof(tiny), &written_len));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, storage_csv::build_row(tiny, sizeof(tiny), &written_len, 1, data));
}
