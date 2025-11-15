# Поки збираємо лише тестову версію
# Базові параметри
set(INCLUDE_DIRS ".")
set(PRIV_REQUIRES_LIST "")

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

idf_component_register(
        SRCS ${SRC_FILES}
        INCLUDE_DIRS ${INCLUDE_DIRS}
        PRIV_REQUIRES ${PRIV_REQUIRES_LIST}
)
