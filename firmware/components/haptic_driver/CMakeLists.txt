idf_component_register(
    SRCS "src/haptic.c" "src/haptic_map.c" "src/haptic_control.c"
    INCLUDE_DIRS "inc"
    PRIV_REQUIRES esp_driver_ledc esp_driver_gpio
)
