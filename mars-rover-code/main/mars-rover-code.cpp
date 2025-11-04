#include "esp_log.h"
#include "hal/ledc_types.h"
#include "motors.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "portmacro.h"
#include <cstdint>

#define SERVO_FREQ 50
#define SERVO_RESOLUTION LEDC_TIMER_14_BIT

extern "C" void app_main(void)
{

    ledc_timer_config_t servo_timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false
    };

    ESP_ERROR_CHECK(ledc_timer_config(&servo_timer_conf));
    // gpio_set_direction();
    
    /*DriveSystem rover{LEDC_TIMER_0, LEDC_TIMER_0};

    while (true) {

        for (float i = -40.0f; i <= 40.0f; i++) {
            rover.rotate(i);
            ESP_LOGI("Angle", "%.2f", i);
            rover.print_angles();
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }

        for (float i = 40.0f; i >= -40.0f; i--) {
            rover.rotate(i);
            ESP_LOGI("Angle", "%.2f", i);
            rover.print_angles();
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }

    }*/
    WheelMotor motor{GPIO_NUM_5, GPIO_NUM_6, "TAG", LEDC_CHANNEL_0};

    WheelMotor::shared_timer = LEDC_TIMER_0;
    while (true) {
        for (uint8_t i = 0; i < 255; i++) {
            motor.forward(255);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            ESP_LOGI("motor", "%d", i);
        }

        for (uint8_t i = 255; i > 0; i--) {
            motor.backward(255);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
}
