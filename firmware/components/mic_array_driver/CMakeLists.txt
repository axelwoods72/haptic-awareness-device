idf_component_register(
    SRC_DIRS "src"
    INCLUDE_DIRS "inc"
    PRIV_REQUIRES esp_driver_i2s esp_driver_gpio board_config freertos espressif__esp-dsp
)