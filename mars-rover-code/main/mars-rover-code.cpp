#include "esp_log.h"
#include "motors.h"
#include "driver/ledc.h"

#define SERVO_GPIO 18
#define SERVO_FREQ 50
#define SERVO_RESOLUTION LEDC_TIMER_14_BIT

extern "C" [[noreturn]] void app_main(void)
{

    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 10000,
        .clk_cfg          = LEDC_AUTO_CLK,
        .deconfigure      = false
    };

    ledc_timer_config_t serv0_timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = SERVO_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));
    ESP_LOGE("LOG", "This is an error");
    // gpio_set_direction();
}
