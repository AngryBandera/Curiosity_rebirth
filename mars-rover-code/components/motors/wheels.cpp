#include "wheels.h"
#include "motors_cfg.h"
//#include "esp_log.h"
#include <cmath>
#include <memory>


FixedWheel::FixedWheel(PCA9685Buffer* buffer,
        uint8_t pca1, uint8_t pca2,
        const char *TAG,
        int32_t Y, int32_t X)
    : buffer{buffer},
      pca1{pca1}, pca2{pca2},
      TAG{TAG},
      Y{Y}, X{X}
{
    spin_radius = static_cast<uint32_t>(
        std::sqrtf(
            static_cast<float>(Y*Y + X*X)
        )
    );
}

void FixedWheel::set_speed(int16_t speed) {

//===========================================
    debug_speed = speed;
//===========================================

    uint16_t pwm = std::abs(speed);

    buffer->set_channel_value(pca1, (speed > 0) ? pwm : 0);
    buffer->set_channel_value(pca2, (speed < 0) ? 0 : pwm);
    // ESP_LOGD(TAG, "speed=%d -> pwm=%u (ch%u/%u)", speed, pwm, pca1, pca2);
}

int32_t FixedWheel::get_X() const {
    return X;
}

int32_t FixedWheel::get_Y() const {
    return Y;
}



SteerableWheel::SteerableWheel(PCA9685Buffer* buffer,
        uint8_t pca1, uint8_t pca2,
        const char *TAG,
        int16_t l, int16_t d,
        uint8_t servo_pca)
    : FixedWheel{buffer, pca1, pca2, TAG, l, d},
        servo_pca{servo_pca}
{
    spin_target_angle = -std::atanf((float)l / (float)d) * 180.0f / PI;
}

void SteerableWheel::set_angle(float angle) {
    angle = std::clamp(angle, -45.0f, 45.0f);

//===========================================
    debug_angle = angle;
//===========================================

    float servo_angle = Cfg::WHEEL_CENTER_ANGLE + angle;
     // max posible - 90 + 45 = 135

    uint32_t pulse_width_us = Servo::MIN_PULSE_US + 
        static_cast<uint32_t>(servo_angle * Servo::DEGREE_TO_US);

    uint16_t duty = static_cast<uint16_t>(
        (Servo::MAX_DUTY * pulse_width_us) / Servo::PERIOD_US
    );

    buffer->set_channel_value(servo_pca, duty);
}
