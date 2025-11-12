# Перемикач для тесту камери
set(TEST_CAMERA_BUILD ON)  # 🔄 Зміни на OFF, щоб зібрати основний код

# Базові параметри
set(INCLUDE_DIRS ".")
set(PRIV_REQUIRES_LIST "")

#if(TEST_CAMERA_BUILD)
message(STATUS "📸 Building TEST_CAMERA project")
set(SRC_FILES "test_camera.cpp")
list(APPEND PRIV_REQUIRES_LIST
        camera
        esp_system

        esp_wifi
        nvs_flash
        esp_event
        esp_netif
)
#else()
#message(STATUS "🚀 Building FULL Mars Rover project")
#set(SRC_FILES "mars-rover-code.cpp")
#list(APPEND PRIV_REQUIRES_LIST
#        camera
#        motors
#        bluetooth
#        espnow_common
#        driver
#        esp_log
#        nvs_flash
#        esp_system
#)
#endif()

idf_component_register(
        SRCS ${SRC_FILES}
        INCLUDE_DIRS ${INCLUDE_DIRS}
        PRIV_REQUIRES ${PRIV_REQUIRES_LIST}
)
