idf_component_register(SRCS "main.cpp" 
                    PRIV_REQUIRES esp_driver_mcpwm bt nvs_flash vfs
                    REQUIRES motors bluepad32 btstack bluetooth stepper
                    INCLUDE_DIRS ".")

