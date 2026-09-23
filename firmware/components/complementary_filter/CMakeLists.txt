idf_component_register (
    SRC_DIRS "src"
    INCLUDE_DIRS "inc"
    REQUIRES imu_driver
    PRIV_REQUIRES esp_timer
)