
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// includes for bluepad32
#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <uni.h>

#include "sdkconfig.h"

// Rover C++ headers
#include "drive_system.h"

static i2c_dev_t* g_pca9685_dev = nullptr;
static DriveSystem* g_rover = nullptr;

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

extern "C" {
    struct uni_platform* get_my_platform(DriveSystem *ds);

    void app_main()
    {
        static const char *TAG = "ROVER_MAIN";

        rover_mutex = xSemaphoreCreateMutex();

        g_pca9685_dev = new i2c_dev_t{};

        g_rover = new DriveSystem(g_pca9685_dev);

        xTaskCreatePinnedToCore(
            rover_tick_task,
            "rover_tick",
            4096,
            NULL,
            10,
            NULL,
            1 
        );

        ESP_LOGI("ROVER_MAIN",
            "Initialization complete. Rover tick task is running.");

        btstack_init();

        uni_platform_set_custom(get_my_platform(g_rover));

        uni_init(0 /* argc */, NULL /* argv */);

        btstack_run_loop_execute();
    }
}
