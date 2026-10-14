idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS ""
    PRIV_REQUIRES imu_driver complementary_filter mic_array_driver
    )
