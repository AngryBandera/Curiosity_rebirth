// C++ headers
#include <cstddef>
#include <cstdint>
#include <sys/types.h>
#include <string.h> 

// C headers
extern "C" {

    #include <string.h>
    #include <stdbool.h>
    #include <stdio.h>
    #include <inttypes.h>
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_log.h"
    #include "time.h"
    #include "sys/time.h"
    #include "sys/unistd.h"
    #include <ctype.h>

    #include <stdlib.h>

    // includes for bluepad32
    #include <btstack_port_esp32.h>
    #include <btstack_run_loop.h>
    #include <btstack_stdio_esp32.h>
    #include <hci_dump.h>
    #include <hci_dump_embedded_stdout.h>
    #include <uni.h>

    #include "sdkconfig.h"

}

// Rover C++ headers
#include "motors.h"
#include "driver/ledc.h"
#include "pca9685.h"

static i2c_dev_t *g_pca9685_dev = nullptr;
static DriveSystem *g_rover = nullptr;

static SemaphoreHandle_t rover_mutex = nullptr;

static void rover_tick_task(void *param)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms
    
    while (1) {
        if (xSemaphoreTake(rover_mutex, portMAX_DELAY)) {
            if (g_rover) {
                g_rover->tick();
            }
            xSemaphoreGive(rover_mutex);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

#define ROVER_MAX_TURN_ANGLE 60.0f
#define ROVER_STOP_SPEED 0

static int max_speed = 1000;

extern "C" {
    struct uni_platform* get_my_platform(DriveSystem *ds);

    void app_main()
    {
        static const char *TAG = "ROVER_MAIN";

        rover_mutex = xSemaphoreCreateMutex();

        g_pca9685_dev = new i2c_dev_t{};

        g_rover = new DriveSystem(g_pca9685_dev);
        ESP_LOGI(TAG, "Rover DriveSystem Initialized.");


        xTaskCreatePinnedToCore(
            rover_tick_task,
            "rover_tick",
            4096,
            NULL,
            10,
            NULL,
            1 
        );

        ESP_LOGI(TAG, "Initialization complete. Rover tick task is running.");

        btstack_init();

        uni_platform_set_custom(get_my_platform(g_rover));
        // uni_platform_set_custom(get_my_platform());

        uni_init(0 /* argc */, NULL /* argv */);

        btstack_run_loop_execute();
    }
}
