idf_component_register(
    SRCS "motors.cpp" "./motors.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_ledc esp_driver_gpio pca9685
)
