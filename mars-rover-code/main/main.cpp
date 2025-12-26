#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
//#include "freertos/task.h"
#include "esp_log.h"

// includes for bluepad32
#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <uni.h>

#include "sdkconfig.h"

// Rover C++ headers
#include "drive_system.h"

static DriveSystem* g_rover = nullptr;
static StepperMotor* camera_stepper = nullptr;

static void rover_tick_task(void *param)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms

    for (;;) {
        if (g_rover) {
            g_rover->tick();
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

extern "C" {
    struct uni_platform* get_my_platform(DriveSystem *ds, StepperMotor* stepper);

    void app_main()
    {
        // Initialize stepper motor for camera pan
        StepperMotor::Config stepper_config = {
            .gpio_en = GPIO_NUM_0,          // Enable pin
            .gpio_dir = GPIO_NUM_27,        // Direction pin
            .gpio_step = GPIO_NUM_26,       // Step pin
            .servo_pin = GPIO_NUM_4,        // Servo pin for vertical tilt
            .enable_level = 0,              // A4988 is enabled on low level
            .resolution_hz = 1000000,       // 1MHz resolution
            .min_speed_hz = 500,            // Minimum speed 500Hz
            .max_speed_hz = 1200,           // Maximum speed 1200Hz (reduced for better control)
            .accel_sample_points = 500      // 500 sample points for acceleration
        };
        
        camera_stepper = new StepperMotor(stepper_config);
        esp_err_t err = camera_stepper->init();
        if (err != ESP_OK) {
            ESP_LOGE("ROVER_MAIN", "Failed to initialize stepper motor");
        } else {
            camera_stepper->set_enabled(true);
            camera_stepper->start_task("stepper_task", 5, 1);
            ESP_LOGI("ROVER_MAIN", "Stepper motor initialized and task started");
        }

        g_rover = DriveSystem::create(GPIO_NUM_21, GPIO_NUM_22);

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

        uni_platform_set_custom(get_my_platform(g_rover, camera_stepper));

        uni_init(0 /* argc */, NULL /* argv */);

        btstack_run_loop_execute();
    }
}
