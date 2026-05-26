#include <string.h>

#include "unity.h"

#include "classifier.h"
#include "storage.h"

TEST_CASE("storage csv header format is stable", "[storage]") {
    char buffer[256] = {};
    size_t written_len = 0;

    const esp_err_t ret = storage_csv::build_header(buffer, sizeof(buffer), &written_len);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    const char *expected = "timestamp_epoch,dht11_temp_c,dht11_humidity_pct,ks0033_temp_c,light_raw,moisture_raw,predicted\n";
    TEST_ASSERT_EQUAL_STRING(expected, buffer);
    TEST_ASSERT_EQUAL(strlen(expected), written_len);
}

TEST_CASE("storage csv row format is stable", "[storage]") {
    SensorData data = {};
    data.dht11_ok           = true;
    data.dht11_temperature_c = 23.456F;
    data.dht11_humidity_pct  = 55.0F;
    data.ks0033_ok           = true;
    data.ks0033_temperature_c = 22.5F;
    data.moisture_ok         = true;
    data.moisture_raw        = 1000;
    data.moisture_mv         = 806;
    data.moisture_pct        = 75.6F;
    data.light_ok            = true;
    data.light_raw           = 2048;
    data.light_mv            = 1650;
    data.light_pct           = 50.0F;

    char buffer[256] = {};
    size_t written_len = 0;

    const esp_err_t ret = storage_csv::build_row(buffer, sizeof(buffer), &written_len,
                                                  1713696000, data, PlantState::Estable);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    const char *expected = "1713696000,23.46,55.00,22.50,2048,1000,Estable\n";
    TEST_ASSERT_EQUAL_STRING(expected, buffer);
    TEST_ASSERT_EQUAL(strlen(expected), written_len);
}

TEST_CASE("storage csv formatting validates arguments", "[storage]") {
    size_t written_len = 0;
    SensorData data = {};

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, storage_csv::build_header(nullptr, 10, &written_len));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, storage_csv::build_row(nullptr, 10, &written_len, 1, data, PlantState::Estable));
}

TEST_CASE("storage csv formatting detects short buffers", "[storage]") {
    char tiny[8] = {};
    size_t written_len = 0;
    SensorData data = {};

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, storage_csv::build_header(tiny, sizeof(tiny), &written_len));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, storage_csv::build_row(tiny, sizeof(tiny), &written_len, 1, data, PlantState::Estable));
}
