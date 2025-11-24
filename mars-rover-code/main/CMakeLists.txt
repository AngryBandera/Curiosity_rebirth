idf_component_register(SRCS "main.cpp" 
                    PRIV_REQUIRES driver esp_driver_mcpwm bt nvs_flash vfs
                    REQUIRES driver motors bluepad32 btstack bluetooth
                    INCLUDE_DIRS ".")

