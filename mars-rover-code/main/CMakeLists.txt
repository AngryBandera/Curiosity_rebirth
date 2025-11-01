idf_component_register(SRCS "main.c"
                            "bluetooth/spp_task.c"
                            "motors/motors.cpp"
                            "servoController/servoController.cpp"
                    PRIV_REQUIRES driver esp_driver_mcpwm bt nvs_flash vfs
                    INCLUDE_DIRS "." "bluetooth" "motors" "servoController")
