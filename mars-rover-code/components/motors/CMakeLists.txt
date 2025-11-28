idf_component_register(
    SRCS "wheel_motor.cpp" "drive_system.cpp" "stepper.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_ledc esp_driver_gpio pca9685 esp_timer
)
