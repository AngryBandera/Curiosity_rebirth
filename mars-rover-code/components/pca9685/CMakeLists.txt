idf_component_register(
    SRCS "pca9685.c" "pca_buffer.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_idf_lib_helpers i2cdev motors
)
