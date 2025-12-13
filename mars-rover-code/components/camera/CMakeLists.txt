idf_component_register(
        SRCS "camera_server.cpp"
        INCLUDE_DIRS "."
        REQUIRES espressif__esp32-camera esp_system nvs_flash esp_http_server esp_timer
)
