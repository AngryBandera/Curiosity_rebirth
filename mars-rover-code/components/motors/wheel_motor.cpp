#include "wheel_motor.h"
#include "esp_err.h"
#include "esp_log.h"
#include "hal/ledc_types.h"
#include "i2cdev.h"
#include "pca9685.h"
#include <cmath>
#include <math.h>
#include <cstdint>
#include <memory>
#include <sys/types.h>


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


// helper
static inline uint16_t clamp_pwm(int32_t v) {
    if (v <= 0) return 0;
    if (v > 4095) return 4095;
    return static_cast<uint16_t>(v);
}


WheelMotor::WheelMotor(uint8_t pca1, uint8_t pca2,
        const char *TAG,
        int16_t l, int16_t d)
    : pca1{pca1}, pca2{pca2},
      TAG{TAG},
      l{l}, d{d} {}

void WheelMotor::update_buffer(int16_t speed, PCA9685Buffer* buffer) {
    int32_t abs_speed = std::abs(static_cast<int32_t>(speed));
    uint16_t pwm = clamp_pwm(static_cast<int32_t>(abs_speed * Cfg::MOTOR_SCALE));

    if (speed > 0) {
        buffer->set_channel_value(pca1, pwm);
        buffer->set_channel_value(pca2, 0);
    } else if (speed < 0) {
        buffer->set_channel_value(pca1, 0);
        buffer->set_channel_value(pca2, pwm);
    } else {
        buffer->set_channel_value(pca1, 0);
        buffer->set_channel_value(pca2, 0);
    }
    ESP_LOGD(TAG, "speed=%d -> pwm=%u (ch%u/%u)", speed, pwm, pca1, pca2);
}


void WheelMotor::update_geometry(int32_t rvr_radius) {
    int32_t offset = rvr_radius - d;
    inner_radius = (uint32_t)abs(offset);
}

uint32_t WheelMotor::get_radius() {
    return inner_radius;
}



SteerableWheel::SteerableWheel(uint8_t pca1, uint8_t pca2,
        const char *TAG,
        int16_t l, int16_t d,
        uint8_t servo_pca)
    : WheelMotor{pca1, pca2, TAG, l, d},
        servo_pca{servo_pca} {}

void SteerableWheel::update_buffer(int16_t speed, PCA9685Buffer* buffer) {
    int32_t abs_speed = std::abs(static_cast<int32_t>(speed));
    uint16_t pwm = clamp_pwm(static_cast<int32_t>(abs_speed * Cfg::MOTOR_SCALE));

    if (speed > 0) {
        buffer->set_channel_value(pca1, pwm);
        buffer->set_channel_value(pca2, 0);
    } else if (speed < 0) {
        buffer->set_channel_value(pca1, 0);
        buffer->set_channel_value(pca2, pwm);
    } else {
        buffer->set_channel_value(pca1, 0);
        buffer->set_channel_value(pca2, 0);
    }

    // servo_duty вже обчислено в update_geometry; тут записуємо без зміни
    buffer->set_channel_value(servo_pca, servo_duty);
    ESP_LOGD(TAG, "speed=%d -> pwm=%u, servo=%u (servo ch %u)", speed, pwm, servo_duty, servo_pca);
}

void SteerableWheel::update_geometry(int32_t rvr_radius) {
    if (abs(rvr_radius) > 16000) {
        current_angle = 0.0f;
        inner_radius = (rvr_radius > 0 ? 16000 : -16000);
    } else {

        int8_t sign = (rvr_radius >= 0) ? 1 : -1;
        //inner_radius = isqrt((uint32_t)(l*l + (rvr_radius - d) * (rvr_radius - d)));

        int32_t l_squared = (int32_t)l * l;
        int32_t offset = rvr_radius - d;
        int32_t offset_squared = offset * offset;

        inner_radius = isqrt((uint32_t)(l_squared + offset_squared));

        float ratio = (float)l / (float)inner_radius;
        if (ratio > 1.0f) ratio = 1.0f;
        if (ratio < -1.0f) ratio = -1.0f;

        float angle_rad = asinf(ratio);
        current_angle = angle_rad * 180.0f / PI;

        current_angle *= sign;

        if (current_angle < -Cfg::WHEEL_MAX_DEVIATION) current_angle = -Cfg::WHEEL_MAX_DEVIATION;
        if (current_angle >  Cfg::WHEEL_MAX_DEVIATION) current_angle =  Cfg::WHEEL_MAX_DEVIATION;
    }

    float servo_angle = Cfg::WHEEL_CENTER_ANGLE + current_angle;
     // max posible - 90 + 45 = 135

    if (servo_angle < 0.0f) servo_angle = 0.0f;
    if (servo_angle > 180.0f) servo_angle = 180.0f;

    uint16_t pulse_width_us = Servo::MIN_PULSE_US + 
        static_cast<uint16_t>(servo_angle * Servo::DEGREE_TO_US);
    // 180.0 * 1000 = 0_180_000 <= 4_294_967_296

    uint16_t duty = (Servo::MAX_DUTY * pulse_width_us) / Servo::PERIOD_US;

    //TODO: return data to write it in common array and send through i2c
    //pca9685_set_pwm_value(pca9685, servo_pca, duty);
    servo_duty = duty;
    //buffer->set_channel_value(servo_pca, duty);
    ESP_LOGI(TAG, "duty: %d", duty);
}

float SteerableWheel::get_angle() {
    return current_angle;
}