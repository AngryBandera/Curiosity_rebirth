idf_component_register(SRCS "stepper_motor_encoder.c" "stepper_motor.cpp"
                       PRIV_REQUIRES esp_driver_rmt esp_driver_gpio
                       INCLUDE_DIRS ".")
