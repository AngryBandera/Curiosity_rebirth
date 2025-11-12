#include "motors.h"
#include "esp_log.h"
#include "hal/ledc_types.h"
#include "i2cdev.h"
#include "pca9685.h"
#include <math.h>
#include <cstdint>
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

WheelMotor::WheelMotor(uint8_t pca1, uint8_t pca2,
        const char *TAG,
        int16_t l, int16_t d)
    : pca1{pca1}, pca2{pca2},
      TAG{TAG},
      l{l}, d{d}
{
    ESP_LOGI(TAG, "Configured pca channels (%d and %d) for motor", pca1, pca2);
}

void WheelMotor::move(int16_t speed, i2c_dev_t* pca9685) {
    if (speed > 0) {
        pca9685_set_pwm_value(pca9685, pca1, speed);
        pca9685_set_pwm_value(pca9685, pca2, 0);
    } else {
        pca9685_set_pwm_value(pca9685, pca1, 0);
        pca9685_set_pwm_value(pca9685, pca2, -speed);
    }
    ESP_LOGI(TAG, "Speed: %d", speed);
}


void WheelMotor::update_radius(int32_t med_radius) {
    int32_t offset = med_radius - d;
    inner_radius = (uint32_t)abs(offset);
}

uint32_t WheelMotor::get_radius() {
    return inner_radius;
}

void WheelMotor::stop(i2c_dev_t* pca9685) {
    pca9685_set_pwm_value(pca9685, pca1, 0);
    pca9685_set_pwm_value(pca9685, pca2, 0);
}




SteerableWheel::SteerableWheel(uint8_t pca1, uint8_t pca2,
        const char *TAG,
        int16_t l, int16_t d,
        uint8_t servo_pca)
    : WheelMotor{pca1, pca2, TAG, l, d},
        servo_pca{servo_pca}
{

}

void SteerableWheel::rotate_on_relative_angle(i2c_dev_t* pca9685) {
    float servo_angle = Cfg::WHEEL_CENTER_ANGLE + current_angle;
     // max posible - 90 + 45 = 135

    if (servo_angle < 0.0f) servo_angle = 0.0f;
    if (servo_angle > 180.0f) servo_angle = 180.0f;

    uint32_t pulse_width_us = Servo::MIN_PULSE_US + 
        (uint32_t)((servo_angle / 180.0f) * (Servo::MAX_PULSE_US - Servo::MIN_PULSE_US));
    // 13500 * 1000 = 0_013_500_000 <= 4_294_967_296

    constexpr uint16_t max_duty = (1 << Servo::RESOLUTION);
    uint16_t duty = (max_duty * pulse_width_us) / Servo::PERIOD_US;

    //TODO: return data to write it in common array and send through i2c
    pca9685_set_pwm_value(pca9685, servo_pca, duty);
    ESP_LOGI(TAG, "duty: %d", duty);
}

void SteerableWheel::update_radius_and_angle(int32_t med_radius, float rvr_angle) {
    if (fabs(rvr_angle) < Cfg::ANGLE_DEVIATION) {
        current_angle = 0.0f;
        inner_radius = 10000;
        return;
    }

    int8_t sign = (med_radius >= 0) ? 1 : -1;
    //inner_radius = isqrt((uint32_t)(l*l + (med_radius - d) * (med_radius - d)));

    int32_t l_squared = (int32_t)l * l;
    int32_t offset = med_radius - d;  // int операція
    int32_t offset_squared = offset * offset;
    
    inner_radius = isqrt((uint32_t)(l_squared + offset_squared));

    float ratio = (float)l / (float)inner_radius;
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < -1.0f) ratio = -1.0f;

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




DriveSystem::DriveSystem(i2c_dev_t* pca9685)
    : pca9685{pca9685},
    right_back{
        5, 4,
        "RightBackWheel",
        Cfg::BACK_Y, Cfg::RIGHT_X,
        0},
    right_middle{
        7, 6, 
       "RightMiddleMotor",
       0, Cfg::RIGHT_X
                    },
    right_front{
        9, 8,
        "RightFrontWheel", 
        Cfg::FRONT_Y, Cfg::RIGHT_X,
        1},

    left_back {
        11, 10,
        "LeftBackWheel",
        Cfg::BACK_Y, Cfg::LEFT_X,
        2},
    left_middle {
        13, 12,
        "LeftMiddleMotor",
        0, Cfg::LEFT_X
                    },
    left_front {
        15, 14,
        "LeftFrontWheel",
        Cfg::FRONT_Y, Cfg::LEFT_X,
        3
    }
{
    all_steerable_wheels[0] = &right_back;
    all_steerable_wheels[1] = &right_front;
    all_steerable_wheels[2] = &left_back;
    all_steerable_wheels[3] = &left_front;

    all_wheels[0] = &right_front;
    all_wheels[1] = &right_middle;
    all_wheels[2] = &right_back;

    all_wheels[3] = &left_front;
    all_wheels[4] = &left_middle;
    all_wheels[5] = &left_back;
}

void DriveSystem::rotate(float rvr_angle) {
    if (fabsf(rvr_angle) < 1.0f) {
        ESP_LOGI("ZERO ANGLE", "rotate(): driving straight (angle %.2f° < 0.5°)", rvr_angle);

        for (SteerableWheel* wheel : all_steerable_wheels) {
            wheel->update_radius_and_angle(10000, 0.0f);
        }
        right_middle.update_radius(10000);
        left_middle.update_radius(10000);

        for (SteerableWheel* wheel : all_steerable_wheels) {
            wheel->rotate_on_relative_angle(pca9685);
        }

        prev_angle = 0.0f;
        return;
    }

    constexpr float PI = 3.14159265f;
    float alpha_rad = rvr_angle * PI / 180.0f;
    int32_t med_radius = (int32_t)(Cfg::FRONT_Y / tanf(alpha_rad));
    // theoreticly there shouldn't be overflow

    for (SteerableWheel* wheel : all_steerable_wheels) {
        wheel->update_radius_and_angle(med_radius, rvr_angle);
        wheel->rotate_on_relative_angle(pca9685);
    }
    right_middle.update_radius(med_radius);
    left_middle.update_radius(med_radius);

    prev_angle = rvr_angle;

}

void DriveSystem::print_angles() {
    ESP_LOGI(TAG, "rightBack: %.2f | rightFront: %.2f | leftBack: %.2f | lefftFront: %.2f",
            right_back.get_angle(), right_front.get_angle(), left_back.get_angle(), left_front.get_angle());
}

void DriveSystem::move(int16_t speed) {
    if (fabsf(prev_angle) <= 1.0f) {
        for (uint8_t i = 0; i < 6; i++) all_wheels[i]->move(speed, pca9685);
    } else {
        uint32_t radii[6];
        for (int i = 0; i < 6; i++) {
            radii[i] = all_wheels[i]->get_radius();
        }

        uint32_t maxR = radii[0];
        for (int i = 1; i < 6; i++) {
            if (radii[i] > maxR)
                maxR = radii[i];
        }

        float rot_speed = static_cast<float>(speed) /  static_cast<float>(maxR);
        ESP_LOGI(TAG, "maxR=%u, speed=%d", maxR, speed);
        for (int i = 0; i < 6; i++) {
            int16_t wheel_speed = static_cast<int16_t>(rot_speed * static_cast<float>(radii[i]));
            ESP_LOGI("DriveSystem", "wheel %d: radius=%u, wheel_speed=%d, rot_speed=%.2f", i, radii[i], wheel_speed, rot_speed);

            all_wheels[i]->move(wheel_speed, pca9685);
        }
    }
}
