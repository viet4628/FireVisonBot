idf_component_register(
    SRCS "motor.c"
    INCLUDE_DIRS "include"
    REQUIRES board_hw esp_driver_ledc driver
)