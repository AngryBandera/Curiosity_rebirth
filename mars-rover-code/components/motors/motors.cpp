#include "motors.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "hal/ledc_types.h"
#include "soc/gpio_num.h"
#include <cstdint>
#include <sys/types.h>

WheelMotor::WheelMotor(gpio_num_t pin_pwm, gpio_num_t pin_dir, const char *new_TAG, ledc_channel_t channel_new, uint16_t l_new, uint16_t d_new)
    : pwm_pin{pin_pwm},
      dir_pin{pin_dir},
      channel{channel_new},
      TAG{new_TAG},
      l{l_new},
      d{d_new}
{
    ledc_channel_config_t channel_config = {
        .gpio_num = pwm_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = shared_timer,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = {.output_invert = 0}
    };
    ledc_channel_config(&channel_config);

    gpio_config_t dir_conf{
        .pin_bit_mask = (1ULL << dir_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&dir_conf);

    gpio_set_level(dir_pin, 0);

    ESP_LOGI(TAG, "Configured pins (%d and %d) for motor", pwm_pin, dir_pin);
}

void WheelMotor::forward(uint8_t speed) {
    gpio_set_level(dir_pin, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, speed);
}

void WheelMotor::backward(uint8_t speed) {
    gpio_set_level(pwm_pin, 1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, 255-speed);
}

void WheelMotor::forward_rot(uint16_t rot_speed) {
    uint16_t speed = rot_speed * inner_radius;
}

void WheelMotor::stop() {
    gpio_set_level(dir_pin, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, 0);
}




SteerableWheel::SteerableWheel(gpio_num_t pin_1, gpio_num_t pin_2,
        const char *TAG_init, ledc_channel_t channel_init,
        uint16_t l_init, uint16_t d_init,
        gpio_num_t servo_pin_init, ledc_channel_t servo_channel_init)
    : WheelMotor{pin_1, pin_2, TAG_init, channel_init, l_init, d_init},
        servo_pin{servo_pin_init},
        servo_channel{servo_channel_init}
{
    ledc_channel_config_t servo_config = {
        .gpio_num = servo_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = servo_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = servo_timer,  // Інший таймер для серво (50Hz)
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = {.output_invert = 0}
    };
    ledc_channel_config(&servo_config);
}



DriveSystem::DriveSystem(ledc_timer_t timer) {
    WheelMotor::shared_timer = timer;
}
