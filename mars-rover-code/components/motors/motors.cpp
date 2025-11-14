#include "motors.h"
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

// Wheel motor: speed is signed in your internal units (e.g. -max_speed..+max_speed).
// We map absolute speed -> PWM (0..4095). If you have a different internal max (e.g. 1000),
// adjust `SCALE` accordingly or pass a max parameter.
#ifndef MOTOR_INTERNAL_MAX
#define MOTOR_INTERNAL_MAX 3000  // <- змінено на 3000 відповідно до запиту
#endif
static const float MOTOR_SCALE = 4095.0f / static_cast<float>(MOTOR_INTERNAL_MAX);

WheelMotor::WheelMotor(uint8_t pca1, uint8_t pca2,
        const char *TAG,
        int16_t l, int16_t d)
    : pca1{pca1}, pca2{pca2},
      TAG{TAG},
      l{l}, d{d} {}

void WheelMotor::update_buffer(int16_t speed, PCA9685Buffer* buffer) {
    int32_t abs_speed = (speed < 0) ? -static_cast<int32_t>(speed) : static_cast<int32_t>(speed);
    uint16_t pwm = clamp_pwm(static_cast<int32_t>(abs_speed * MOTOR_SCALE));

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


void WheelMotor::update_geometry(int32_t med_radius) {
    int32_t offset = med_radius - d;
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
    int32_t abs_speed = (speed < 0) ? -static_cast<int32_t>(speed) : static_cast<int32_t>(speed);
    uint16_t pwm = clamp_pwm(static_cast<int32_t>(abs_speed * MOTOR_SCALE));

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

void SteerableWheel::update_geometry(int32_t med_radius) {
    if (abs(med_radius) > 16000) {
        current_angle = 0.0f;
        inner_radius = (med_radius > 0 ? 16000 : -16000);
    } else {

        int8_t sign = (med_radius >= 0) ? 1 : -1;
        //inner_radius = isqrt((uint32_t)(l*l + (med_radius - d) * (med_radius - d)));

        int32_t l_squared = (int32_t)l * l;
        int32_t offset = med_radius - d;
        int32_t offset_squared = offset * offset;

        inner_radius = isqrt((uint32_t)(l_squared + offset_squared));

        float ratio = (float)l / (float)inner_radius;
        if (ratio > 1.0f) ratio = 1.0f;
        if (ratio < -1.0f) ratio = -1.0f;

        float angle_rad = asinf(ratio);
        current_angle = angle_rad * 180.0f / Cfg::PI;

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




DriveSystem::DriveSystem(i2c_dev_t* pca9685)
    : buffer{new PCA9685Buffer{pca9685}},
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
    },
    mem_speed{0}, mem_angle{0.0f}, dest_speed{0}, dest_angle{0.0f} // <<< ІНІЦІАЛІЗАЦІЯ
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

void DriveSystem::print_angles() {
    ESP_LOGI(TAG, "rightBack: %.2f | rightFront: %.2f | leftBack: %.2f | lefftFront: %.2f",
            right_back.get_angle(), right_front.get_angle(), left_back.get_angle(), left_front.get_angle());
}

void DriveSystem::actual_move(int16_t speed, float rvr_angle) {
    int32_t med_radius;
    if (fabsf(rvr_angle) <= 1.0f) {
        med_radius = 16001;
    } else {
        float alpha_rad = rvr_angle * Cfg::PI / 180.0f;
        med_radius = static_cast<int32_t>(Cfg::FRONT_Y / tanf(alpha_rad));
    }
    for (uint8_t i = 0; i < 6; i++) all_wheels[i]->update_geometry(med_radius);

    if (fabsf(rvr_angle) <= 1.0f) {

        for (uint8_t i = 0; i < 6; i++) all_wheels[i]->update_buffer(speed, buffer);

    } else {

        uint32_t radii[6];
        for (int i = 0; i < 6; i++) radii[i] = all_wheels[i]->get_radius();

        uint32_t maxR = radii[0];
        for (int i = 1; i < 6; i++)
            if (radii[i] > maxR)
                maxR = radii[i];

        float rot_speed = static_cast<float>(speed) /  static_cast<float>(maxR);
        //ESP_LOGI(TAG, "maxR=%u, speed=%d", maxR, speed);
        for (int i = 0; i < 6; i++) {
            int16_t wheel_speed = static_cast<int16_t>(rot_speed * static_cast<float>(radii[i]));
            //ESP_LOGI("DriveSystem", "wheel %d: radius=%u, wheel_speed=%d, rot_speed=%.2f", i, radii[i], wheel_speed, rot_speed);

            all_wheels[i]->update_buffer(wheel_speed, buffer);
        }
    }

    buffer->flush();
}


void DriveSystem::set(int16_t speed, float rvr_angle) {
    dest_speed = speed;
    dest_angle = rvr_angle;
}

void DriveSystem::set_speed(int16_t speed) {
    dest_speed = speed;
}

void DriveSystem::set_angle(float rvr_angle) {
    dest_angle = rvr_angle;
}


void DriveSystem::tick() {
    if (dest_speed >= 0) {
        if     (dest_speed <= 100) { mem_speed = 0; }
        else if      (dest_speed > mem_speed) { mem_speed += Cfg::DC_ACCEL; }
        else if (dest_speed < mem_speed) { mem_speed -= Cfg::DC_ACCEL * 4; }
    } else {
        if     (dest_speed >= -100) { mem_speed = 0; }
        else if      (dest_speed > mem_speed) { mem_speed += Cfg::DC_ACCEL * 4; }
        else if (dest_speed < mem_speed) { mem_speed -= Cfg::DC_ACCEL; }
    }

    if (fabs(dest_angle - mem_angle) <= 1.0f) { mem_angle = dest_angle; }
    else if (dest_angle > mem_angle) { mem_angle += Cfg::SERVO_ACCECL; }
    else if (dest_angle < mem_angle) { mem_angle -= Cfg::SERVO_ACCECL; }

    actual_move(mem_speed, mem_angle);
}

void DriveSystem::stop() {
    buffer->clear();
    buffer->flush();
}





PCA9685Buffer::PCA9685Buffer(i2c_dev_t* pca9685):
    device{pca9685},
    dirty{false}
{
    ESP_ERROR_CHECK(i2cdev_init());

    memset(device, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(pca9685_init_desc(device, PCA9685_ADDR, 
                                    I2C_NUM_0, 
                                    I2C_MASTER_SDA_IO, 
                                    I2C_MASTER_SCL_IO));

    ESP_ERROR_CHECK(pca9685_init(device));
    ESP_ERROR_CHECK(pca9685_set_pwm_frequency(device, 50));

    clear();
    flush();
}

void PCA9685Buffer::set_channel_value(uint8_t channel, uint16_t value) {
    if (channel >= 16) {
        ESP_LOGE(TAG, "Invalid channel %d", channel);
        return;
    }
    if (value > 4095) {
        ESP_LOGW(TAG, "clamping PWM %u->4095 for channel %u", value, channel);
        value = 4095;
    }
    buffer[channel] = value;
    dirty = true;
}

uint16_t PCA9685Buffer::get_channel_value(uint8_t channel) {
    if (channel >= 16) {
        ESP_LOGE(TAG, "Invalid channel %d", channel);
        return 0;
    }
    return buffer[channel];
}

void PCA9685Buffer::flush() {
    // виправлено: якщо нема змін — нічого не робимо; інакше — шлемо
    if (!dirty) return;
    ESP_ERROR_CHECK(pca9685_set_pwm_values(device,
            0, 16,
            buffer));
    dirty = false;
}

void PCA9685Buffer::set_channel_immediate(uint8_t channel, uint16_t value) {
    ESP_ERROR_CHECK(pca9685_set_pwm_value(device, channel, value));
}

bool PCA9685Buffer::is_dirty() const {
    return dirty;
}

void PCA9685Buffer::clear() {
    for (uint8_t i = 0; i < 4; i++)  buffer[i] = Servo::CENTER_DUTY;
    for (uint8_t i = 4; i < 16; i++) buffer[i] = 0;

    dirty = true;
}
