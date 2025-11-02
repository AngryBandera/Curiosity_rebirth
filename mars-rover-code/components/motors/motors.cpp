#include "motors.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "hal/ledc_types.h"
#include "soc/gpio_num.h"
#include <math.h>
#include <cstdint>
#include <sys/types.h>


ledc_timer_t WheelMotor::shared_timer = LEDC_TIMER_0;
ledc_timer_t SteerableWheel::servo_timer = LEDC_TIMER_0;

uint32_t isqrt(uint32_t n) {
    if (n == 0) return 0;
    uint32_t x = n;
    uint32_t y = (x + 1) / 2;
    while (y < x) {
        x = y;
        y = (x + n / x) / 2;
    }
    return x;
}

WheelMotor::WheelMotor(gpio_num_t pin_pwm, gpio_num_t pin_dir,
        const char *new_TAG, ledc_channel_t channel_new,
            int16_t l_new, int16_t d_new)
    : pwm_pin{pin_pwm},
      dir_pin{pin_dir},
      channel{channel_new},
      TAG{new_TAG},
      l{l_new},
      d{d_new}
{
    /*ledc_channel_config_t channel_config = {
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
    ledc_channel_config(&channel_config);*/

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
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, speed));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

void WheelMotor::backward(uint8_t speed) {
    gpio_set_level(dir_pin, 1);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, 255-speed));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

void WheelMotor::forward_rot(uint8_t speed, int32_t med_radius) {
    uint8_t new_speed = (uint8_t)(((uint32_t)speed * inner_radius) / (uint32_t)med_radius);
    gpio_set_level(dir_pin, 0);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, new_speed));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

void WheelMotor::backward_rot(uint8_t speed, int32_t med_radius) {
    uint8_t new_speed = (uint8_t)(((uint32_t)speed * inner_radius) / (uint32_t)med_radius);
    gpio_set_level(dir_pin, 1);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, 255-new_speed));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

void WheelMotor::update_radius(int32_t med_radius) {
    int32_t offset = med_radius - d;
    inner_radius = (uint32_t)abs(offset);
}

uint32_t WheelMotor::get_radius() {
    return inner_radius;
}

void WheelMotor::stop() {
    gpio_set_level(dir_pin, 0);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));

}




SteerableWheel::SteerableWheel(gpio_num_t pin_1, gpio_num_t pin_2,
        const char *TAG_init, ledc_channel_t channel_init,
        int16_t l_init, int16_t d_init,
        gpio_num_t servo_pin_init, ledc_channel_t servo_channel_init)
    : WheelMotor{pin_1, pin_2, TAG_init, channel_init, l_init, d_init},
        servo_pin{servo_pin_init},
        servo_channel{servo_channel_init}
{
    uint32_t max_duty = (1 << Servo::RESOLUTION) - 1;
    uint32_t duty_90 = Servo::MIN_PULSE_US + (Servo::MAX_PULSE_US - Servo::MIN_PULSE_US) / 2;
    duty_90 = duty_90 * max_duty / Servo::PERIOD_US;

    ledc_channel_config_t servo_config = {
        .gpio_num = servo_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = servo_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = servo_timer,  // timer for servo (50Hz)
        .duty = duty_90,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = {.output_invert = 0}
    };
    ledc_channel_config(&servo_config);
}

void SteerableWheel::rotate_on_relative_angle() {
    float servo_angle = Cfg::WHEEL_CENTER_ANGLE + current_angle;
     // max posible - 9000 + 4500 = 13500

    if (servo_angle < 0.0f) servo_angle = 0.0f;
    if (servo_angle > 180.0f) servo_angle = 180.0f;

    uint32_t pulse_width_us = Servo::MIN_PULSE_US + 
        (uint32_t)((servo_angle / 180.0f) * (Servo::MAX_PULSE_US - Servo::MIN_PULSE_US));
    // 13500 * 1000 = 0_013_500_000 <= 4_294_967_296

    constexpr uint32_t max_duty = (1 << Servo::RESOLUTION) - 1;
    uint32_t duty = (max_duty * pulse_width_us) / Servo::PERIOD_US;

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, servo_channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, servo_channel));
}

void SteerableWheel::update_radius_and_angle(int32_t med_radius, float rvr_angle) {
    if (fabs(rvr_angle) < Cfg::ANGLE_DEVIATION) {
        current_angle = 0.0f;
        inner_radius = (1 << 31);
        return;
    }

    int8_t sign = (med_radius >= 0) ? 1 : -1;
    //inner_radius = isqrt((uint32_t)(l*l + (med_radius - d) * (med_radius - d)));

    int32_t l_squared = (int32_t)l * l;
    int32_t offset = med_radius - d;  // int операція
    int32_t offset_squared = offset * offset;
    
    inner_radius = isqrt((uint32_t)(l_squared + offset_squared));

    float ratio = (float)l / (float)inner_radius;
    //if (ratio > 1.0f) ratio = 1.0f;
    //if (ratio < -1.0f) ratio = -1.0f;

    float angle_rad = asinf(ratio);
    current_angle = angle_rad * 180.0f / Cfg::PI;

    current_angle *= sign;

    if (current_angle < -Cfg::WHEEL_MAX_DEVIATION) {
        current_angle = -Cfg::WHEEL_MAX_DEVIATION;
    }
    if (current_angle > Cfg::WHEEL_MAX_DEVIATION) {
        current_angle = Cfg::WHEEL_MAX_DEVIATION;
    }
}

float SteerableWheel::get_angle() {
    return current_angle;
}




DriveSystem::DriveSystem(ledc_timer_t timer, ledc_timer_t servo_timer)
    : right_back{
        GPIO_NUM_4, GPIO_NUM_5,
        "RightBackWheel", LEDC_CHANNEL_0,
        Cfg::BACK_Y, Cfg::RIGHT_X,
        GPIO_NUM_6, LEDC_CHANNEL_1},
    //right_middle{
    //    GPIO_NUM_4, GPIO_NUM_5, 
    //   "RightMiddleMotor", LEDC_CHANNEL_3},
    right_front{
        GPIO_NUM_7, GPIO_NUM_15,
        "RightFrontWheel", LEDC_CHANNEL_2,
        Cfg::FRONT_Y, Cfg::RIGHT_X,
        GPIO_NUM_16, LEDC_CHANNEL_3},
    left_back {
        GPIO_NUM_17, GPIO_NUM_18,
        "LeftBackWheel", LEDC_CHANNEL_4,
        Cfg::BACK_Y, Cfg::LEFT_X,
        GPIO_NUM_8, LEDC_CHANNEL_5},
    //left_middle {
    //    GPIO_NUM_4, GPIO_NUM_5,
    //    "LeftMiddleMotor", LEDC_CHANNEL_0},
    left_front {
        GPIO_NUM_3, GPIO_NUM_9,
        "LeftFrontWheel", LEDC_CHANNEL_6,
        Cfg::FRONT_Y, Cfg::LEFT_X,
        GPIO_NUM_10, LEDC_CHANNEL_7
    }
{
    WheelMotor::shared_timer = timer;
    SteerableWheel::servo_timer = servo_timer;
}

void DriveSystem::rotate(float rvr_angle) {
    if (fabsf(rvr_angle - prev_angle) < Cfg::ANGLE_DEVIATION) {
        ESP_LOGI(TAG, "rotate(): angle unchanged (%.2f°)", rvr_angle);
        return;
    }

    if (fabsf(rvr_angle) < 1.0f) {
        ESP_LOGI("ZERO ANGLE", "rotate(): driving straight (angle %.2f° < 0.5°)", rvr_angle);

        right_front.update_radius_and_angle(10000, 0.0f);
        right_back.update_radius_and_angle(10000, 0.0f);
        left_front.update_radius_and_angle(10000, 0.0f);
        left_back.update_radius_and_angle(10000, 0.0f);

        right_front.rotate_on_relative_angle();
        right_back.rotate_on_relative_angle();
        left_front.rotate_on_relative_angle();
        left_back.rotate_on_relative_angle();

        prev_angle = 0.0f;
        return;
    }

    constexpr float PI = 3.14159265f;
    float alpha_rad = rvr_angle * PI / 180.0f;
    int32_t med_radius = (int32_t)(Cfg::FRONT_Y / tanf(alpha_rad));
    // theoreticly there shouldn't be overflow

    right_front.update_radius_and_angle(med_radius, rvr_angle);
    right_front.rotate_on_relative_angle();

    right_back.update_radius_and_angle(med_radius, rvr_angle);
    right_back.rotate_on_relative_angle();

    left_front.update_radius_and_angle(med_radius, rvr_angle);
    left_front.rotate_on_relative_angle();

    left_back.update_radius_and_angle(med_radius, rvr_angle);
    left_back.rotate_on_relative_angle();

    prev_angle = rvr_angle;

}

void DriveSystem::print_angles() {
    ESP_LOGI(TAG, "rightBack: %.2f | rightFront: %.2f | leftBack: %.2f | lefftFront: %.2f",
            right_back.get_angle(), right_front.get_angle(), left_back.get_angle(), left_front.get_angle());
}
