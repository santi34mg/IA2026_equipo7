idf_component_register(
    SRCS
        "main.cpp"
        "sensor.cpp"
        "storage.cpp"
        "test_storage.cpp"
        "time_service.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio
        esp_driver_i2c
        esp_timer
        esp_adc
        spiffs
        nvs_flash
        unity
)
