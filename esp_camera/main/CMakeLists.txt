# Опція: що збираємо
option(Build_FULL_Project "Build full ESP-CAM project" ON)

set(INCLUDE_DIRS ".")
set(PRIV_REQUIRES_LIST "")

if(Build_FULL_Project)
    message(STATUS "Building FULL_CAM project")
    set(SRC_FILES 
        esp_cam_main.cpp 
        command_get.cpp
        )
else()
    message(STATUS "Building TEST_CAMERA project")
    set(SRC_FILES "test_camera.cpp")
endif()

list(APPEND PRIV_REQUIRES_LIST
        camera
        esp_system
        esp_wifi
        nvs_flash
        esp_event
        esp_netif
        esp_http_client
        log
)

idf_component_register(
        SRCS ${SRC_FILES}
        INCLUDE_DIRS ${INCLUDE_DIRS}
        PRIV_REQUIRES ${PRIV_REQUIRES_LIST}
)
