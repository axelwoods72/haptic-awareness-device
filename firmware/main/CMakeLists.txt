idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS ""
    PRIV_REQUIRES imu_driver complementary_filter mic_array_driver board_config haptic_driver esp_timer esp_driver_ledc
    )
