idf_component_register(
    SRCS "./motors.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_ledc esp_driver_gpio pca9685
)
