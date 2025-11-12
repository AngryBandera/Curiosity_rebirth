idf_component_register(SRCS "main.cpp"
                            "bluetooth/spp_task.c"
                    PRIV_REQUIRES driver esp_driver_mcpwm bt nvs_flash vfs
                    REQUIRES driver motors pca9685
                    INCLUDE_DIRS "." "bluetooth")

