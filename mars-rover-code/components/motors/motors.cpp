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
    int32_t abs_speed = std::abs(static_cast<int32_t>(speed));
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
    int32_t abs_speed = std::abs(static_cast<int32_t>(speed));
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
    mem_speed{0}, mem_angle{0.0f}, dest_speed{0}, dest_angle{0.0f}, actual_speed{0.0f} // <<< ІНІЦІАЛІЗАЦІЯ
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

bool DriveSystem::is_moving() {
    return mem_speed == 0 && mem_angle == 0;
}

bool DriveSystem::should_change_direction() const {
    return (mem_speed > 0 && dest_speed < -100) || 
           (mem_speed < 0 && dest_speed > 100);
}

bool DriveSystem::should_start_turning() const {
    return (fabsf(dest_angle - mem_angle) > 5.0f) && 
           (fabsf(dest_angle) > 10.0f);
}

void DriveSystem::handle_idle() {
    // Нічого не робимо, всі мотори вимкнені
    if (previous_state != DriveState::IDLE) {
        stop();
    }
}

void DriveSystem::handle_accelerating() {
    // Звичайний розгін
    if (dest_speed > mem_speed) {
        mem_speed = std::min(dest_speed, 
                             static_cast<int16_t>(mem_speed + Cfg::DC_ACCEL));
    } else if (dest_speed < mem_speed) {
        mem_speed = std::max(dest_speed, 
                             static_cast<int16_t>(mem_speed - Cfg::DC_ACCEL));
    }
    
    // Плавне обертання керованих коліс
    if (fabsf(dest_angle - mem_angle) > 1.0f) {
        if (dest_angle > mem_angle) {
            mem_angle += Cfg::SERVO_ACCECL;
        } else {
            mem_angle -= Cfg::SERVO_ACCECL;
        }
    } else {
        mem_angle = dest_angle;
    }
}

void DriveSystem::handle_moving() {
    // Слідкуємо за невеликими змінами швидкості
    if (dest_speed > mem_speed) {
        mem_speed = std::min(dest_speed, 
                             static_cast<int16_t>(mem_speed + Cfg::DC_ACCEL / 2));
    } else if (dest_speed < mem_speed) {
        mem_speed = std::max(dest_speed, 
                             static_cast<int16_t>(mem_speed - Cfg::DC_ACCEL / 2));
    }
    
    // Обертання коліс без сильних ривків
    if (fabsf(dest_angle - mem_angle) > 1.0f) {
        if (dest_angle > mem_angle) {
            mem_angle += Cfg::SERVO_ACCECL;
        } else {
            mem_angle -= Cfg::SERVO_ACCECL;
        }
    }
}

void DriveSystem::handle_decelerating() {
    // Агресивне гальмування
    int16_t decel_rate = Cfg::DC_ACCEL * 4;
    
    if (mem_speed > 0) {
        mem_speed = std::max(static_cast<int16_t>(0), 
                             static_cast<int16_t>(mem_speed - decel_rate));
    } else if (mem_speed < 0) {
        mem_speed = std::min(static_cast<int16_t>(0), 
                             static_cast<int16_t>(mem_speed + decel_rate));
    }
}

void DriveSystem::handle_turning() {
    // Зменшуємо швидкість при розвороті коліс
    float turn_percentage = fabsf(dest_angle) / Cfg::WHEEL_MAX_DEVIATION;
    int16_t max_turn_speed = static_cast<int16_t>(
        MOTOR_INTERNAL_MAX * (1.0f - turn_percentage * 0.4f)  // Зменшення на 40% за максимального повороту
    );
    
    if (std::abs(mem_speed) > max_turn_speed) {
        if (mem_speed > 0) mem_speed = std::max(max_turn_speed,                        static_cast<int16_t>(mem_speed - Cfg::DC_ACCEL * 3));
        else               mem_speed = std::min(static_cast<int16_t>(-max_turn_speed), static_cast<int16_t>(mem_speed + Cfg::DC_ACCEL * 3));
    } else if (std::abs(dest_speed) > max_turn_speed) {
        if (dest_speed > 0) mem_speed = std::min(static_cast<int16_t>(mem_speed + Cfg::DC_ACCEL),                       max_turn_speed );
        else                mem_speed = std::max(static_cast<int16_t>(mem_speed - Cfg::DC_ACCEL), static_cast<int16_t>(-max_turn_speed));
    }
    
    // Швидкіше обертаємо колеса при повороті
    constexpr float servo_rate = Cfg::SERVO_ACCECL * 2.0f;
    if (dest_angle > mem_angle) {
        mem_angle += std::max(servo_rate, (dest_angle - mem_angle) * 0.2f);
    } else {
        mem_angle -= std::max(servo_rate, (mem_angle - dest_angle) * 0.2f);
    }
}

void DriveSystem::handle_stopping() {
    // Критичне гальмування — запускаємо механізм інерції
    if (mem_speed != 0) {
        // Зупиняємо мотори (PWM = 0)
        // Але дозволяємо машині ковзати через інерцію
        
        // Обчислюємо час ковзання залежно від швидкості
        // Формула: час = |швидкість| * INERTIA_TICKS_PER_UNIT
        uint16_t calculated_ticks = static_cast<uint16_t>(
            std::abs(mem_speed) * Cfg::INERTIA_TICKS_PER_UNIT
        );
        
        // Обмежуємо максимальним часом
        inertia_ticks_remaining = std::min(calculated_ticks, Cfg::MAX_INERTIA_TICKS);
        inertia_speed = mem_speed;
        
        // Зупиняємо мотори негайно
        actual_speed = static_cast<float>(mem_speed);
        mem_speed = 0;
        
        ESP_LOGI(TAG, "STOPPING: Inertia time = %u ticks (~%.1f sec)", 
                 inertia_ticks_remaining, inertia_ticks_remaining * 0.01f);
    }

    if (actual_speed > 0.0f) actual_speed -= Cfg::UINT_PER_INERTIA_TICKS; //DEBUG
    else                     actual_speed += Cfg::UINT_PER_INERTIA_TICKS; //DEBUG
    //DEBUG: test wether this estimation will work out fine
    
    // // Випрямляємо колеса при екстреному гальмуванні
    // if (fabsf(mem_angle) > 2.0f) {
    //     mem_angle *= 0.8f;
    // } else {
    //     mem_angle = 0.0f;
    // }

    // Do not change mem_angle here to keep last steering position
    // Only after stop we can continue to steer them
}

void DriveSystem::apply_inertia() {
    // Застосовуємо ефект інерції — повільне гальмування
    if (inertia_ticks_remaining > 0) {
        inertia_ticks_remaining--;
        
        // Зменшуємо inertia_speed лінійно залежно від часу, що залишився
        // На початку full_inertia_ticks, в кінці 0
        float progress = static_cast<float>(inertia_ticks_remaining) / 
                        (std::abs(inertia_speed) * Cfg::INERTIA_TICKS_PER_UNIT);
        
        // Це буде значення швидкості, на якому машина "ковзає"
        // На логування (не використовується на моторах)
        ESP_LOGD(TAG, "Inertia: %.2f%% | remaining ticks: %u", 
                 progress * 100.0f, inertia_ticks_remaining);
    }
}

void DriveSystem::update_state() {
    previous_state = current_state;
    state_tick_counter++;
    
    // Логіка переходів між станами
    switch (current_state) {
        case DriveState::IDLE:
            if (is_spinning) {
                // Якщо натиснута кнопка spin, переходимо в режим гальмування перед обертанням
                current_state = DriveState::STOPPING;
                state_tick_counter = 0;
            } else if (std::abs(dest_speed) > 50 || fabsf(dest_angle) > 1.0f) {
                current_state = DriveState::ACCELERATING;
                state_tick_counter = 0;
            }
            break;
            
        case DriveState::ACCELERATING:
            if (is_spinning || dest_speed == 0) {
                current_state = DriveState::STOPPING;
                state_tick_counter = 0;
            } else if (mem_speed == dest_speed) {
                current_state = DriveState::MOVING;
                state_tick_counter = 0;
            } else if (should_change_direction()) {
                current_state = DriveState::STOPPING;
                state_tick_counter = 0;
            } else if (std::abs(mem_speed) > std::abs(dest_speed)) {
                current_state = DriveState::DECELERATING;
                state_tick_counter = 0;
            }
            break;
            
        case DriveState::MOVING:
            if (is_spinning) {
                current_state = DriveState::STOPPING;
                state_tick_counter = 0;
            } else if (should_change_direction()) {
                current_state = DriveState::STOPPING;
                state_tick_counter = 0;
            } else if (should_start_turning()) {
                current_state = DriveState::TURNING;
                state_tick_counter = 0;
            } else if (std::abs(mem_speed) > std::abs(dest_speed)) {
                current_state = DriveState::DECELERATING;
                state_tick_counter = 0;
            } else if (std::abs(mem_speed) < std::abs(dest_speed)) {
                current_state = DriveState::ACCELERATING;
                state_tick_counter = 0;
            }
            break;
            
        case DriveState::DECELERATING:
            if (is_spinning) {
                current_state = DriveState::STOPPING;
                state_tick_counter = 0;
            } else if (dest_speed == 0) {
                current_state = DriveState::STOPPING;
                state_tick_counter = 0;
            } else if (mem_speed == dest_speed) {
                current_state = DriveState::MOVING;
                state_tick_counter = 0;
            } else if (should_change_direction()) {
                current_state = DriveState::STOPPING;
                state_tick_counter = 0;
            } else if (std::abs(dest_speed) > std::abs(mem_speed)) {
                current_state = DriveState::ACCELERATING;
                state_tick_counter = 0;
            }
            break;
            
        case DriveState::TURNING:
            if (is_spinning) {
                current_state = DriveState::STOPPING;
                state_tick_counter = 0;
            } else if (dest_speed == 0 || should_change_direction()) {
                current_state = DriveState::STOPPING;
                state_tick_counter = 0;
            } else if (fabsf(dest_angle) <= 5.0f && fabs(mem_angle) <= 5.0f) {
                current_state = DriveState::MOVING;
                state_tick_counter = 0;
            }
            break;
            
        case DriveState::STOPPING:
            if (inertia_ticks_remaining > 0) {
                if (is_spinning) {
                    // Продовжуємо гальмувати перед обертанням
                    break;
                } else if (dest_speed > actual_speed && dest_speed > 100) {
                    current_state = DriveState::TURNING;
                    state_tick_counter = 0;
                } else if (dest_speed < actual_speed && dest_speed < -100) {
                    current_state = DriveState::TURNING;
                    state_tick_counter = 0;
                }
                break;
            }

            if (is_spinning) {
                // Машина зупинилась, переходимо до обертання
                current_state = DriveState::SPINNING;
                state_tick_counter = 0;
            } else if (!is_spinning) {
                // Звичайне гальмування завершено
                current_state = DriveState::IDLE;
                state_tick_counter = 0;
            }
            break;

        case DriveState::SPINNING:
            if (!is_spinning) {
                // Якщо вимкнули режим обертання, сортуємо колеса і повертаємось до звичайного режиму
                current_state = DriveState::IDLE;
                state_tick_counter = 0;
                mem_speed = 0;
                mem_angle = 0.0f;
            }
            break;
    }
    
    last_dest_speed = dest_speed;
    last_dest_angle = dest_angle;
}

void DriveSystem::tick() {
    update_state();
    
    switch (current_state) {
        case DriveState::IDLE:
            handle_idle();
            break;
        case DriveState::ACCELERATING:
            handle_accelerating();
            break;
        case DriveState::MOVING:
            handle_moving();
            break;
        case DriveState::DECELERATING:
            handle_decelerating();
            break;
        case DriveState::TURNING:
            handle_turning();
            break;
        case DriveState::STOPPING:
            handle_stopping();
            break;
        case DriveState::SPINNING:
            handle_spinning();
            break;
    }
    
    // Застосовуємо інерцію ПІСЛЯ обробки стану
    apply_inertia();
    
    // При обертанні використовуємо спеціальні серво-значення
    if (current_state == DriveState::SPINNING) {
        // Мотори рухаються для обертання навколо осі
        for (int i = 0; i < 4; i++) {
            all_steerable_wheels[i]->update_buffer(mem_speed, buffer);
        }
        buffer->flush();
    } else {
        // Звичайний режим руху
        actual_move(mem_speed, mem_angle);
    }
}

void DriveSystem::print_state() {
    const char* state_names[] = {
        "IDLE", "ACCELERATING", "MOVING", "DECELERATING", "TURNING", "STOPPING", "SPINNING"
    };
    ESP_LOGI(TAG, "State: %s | Speed: %d/%d | Angle: %.1f/%.1f | Inertia: %u | Spinning: %s",
            state_names[static_cast<int>(current_state)],
            mem_speed, dest_speed,
            mem_angle, dest_angle,
            inertia_ticks_remaining,
            is_spinning ? "YES" : "NO");
}

void DriveSystem::stop() {
    buffer->clear();
    buffer->flush();
}

// === МЕТОДИ STEERABLE WHEEL ДЛЯ SPIN РЕЖИМУ ===
void SteerableWheel::calculate_spin_duty(float spin_angle) {
    spin_target_angle = spin_angle;
    
    float servo_angle = Cfg::WHEEL_CENTER_ANGLE + spin_angle;
    
    if (servo_angle < 0.0f) servo_angle = 0.0f;
    if (servo_angle > 180.0f) servo_angle = 180.0f;
    
    uint16_t pulse_width_us = Servo::MIN_PULSE_US + 
        static_cast<uint16_t>(servo_angle * Servo::DEGREE_TO_US);
    
    spin_servo_duty = (Servo::MAX_DUTY * pulse_width_us) / Servo::PERIOD_US;
}

// === НОВІ МЕТОДИ DRIVE SYSTEM ===
void DriveSystem::set_spin_input(int16_t throttle, int16_t brake) {
    spin_input_throttle = throttle;
    spin_input_brake = brake;
    
    // Якщо хоча б одна кнопка натиснута, переходимо в режим обертання
    if (throttle > 50 || brake > 50) {
        is_spinning = true;
    }
}

void DriveSystem::stop_spinning() {
    is_spinning = false;
    spin_input_throttle = 0;
    spin_input_brake = 0;
}

void DriveSystem::handle_spinning() {
    // Обчислюємо швидкість обертання з різниці дроселя і гальма
    int16_t spin_speed_input = spin_input_throttle - spin_input_brake;
    
    // Нормалізуємо до діапазону [-SPIN_MAX_SPEED, SPIN_MAX_SPEED]
    float normalized = (float)spin_speed_input / 512.0f;  // 512 - половина осі
    int16_t spin_speed = static_cast<int16_t>(normalized * Cfg::SPIN_MAX_SPEED);
    
    // Плавне наближення до потрібної швидкості обертання
    if (spin_speed > mem_speed) {
        mem_speed = std::min(spin_speed, 
                            static_cast<int16_t>(mem_speed + Cfg::DC_ACCEL));
    } else if (spin_speed < mem_speed) {
        mem_speed = std::max(spin_speed, 
                            static_cast<int16_t>(mem_speed - Cfg::DC_ACCEL));
    }
    
    // Плавне обертання керованих коліс до потрібних кутів
    // Передні колеса
    if (fabsf(Cfg::SPIN_FRONT_ANGLE - right_front.spin_target_angle) > 0.5f) {
        left_front.calculate_spin_duty(
            // -right_front.spin_target_angle
            left_front.spin_target_angle + (Cfg::SPIN_FRONT_ANGLE - left_front.spin_target_angle) * 0.1f
        );
        right_front.calculate_spin_duty(
            -left_front.spin_target_angle //FIX: theoreticly should be symmetrical
            // right_front.spin_target_angle + (Cfg::SPIN_FRONT_ANGLE - right_front.spin_target_angle) * 0.1f
        );
    } else {
        left_front.calculate_spin_duty(Cfg::SPIN_FRONT_ANGLE);
        right_front.calculate_spin_duty(-Cfg::SPIN_FRONT_ANGLE);
    }
    
    // Задні колеса
    if (fabsf(Cfg::SPIN_BACK_ANGLE - right_back.spin_target_angle) > 0.5f) {
        right_back.calculate_spin_duty(
            right_back.spin_target_angle + (Cfg::SPIN_BACK_ANGLE - right_back.spin_target_angle) * 0.1f
        );
        left_back.calculate_spin_duty(
            -right_back.spin_target_angle //FIX: theoreticly should be symmetrical
            // left_back.spin_target_angle + (Cfg::SPIN_BACK_ANGLE - left_back.spin_target_angle) * 0.1f
        );
    } else {
        left_back.calculate_spin_duty(-Cfg::SPIN_BACK_ANGLE);
        right_back.calculate_spin_duty(Cfg::SPIN_BACK_ANGLE);
    }
    
    ESP_LOGD(TAG, "SPINNING: speed=%d, throttle=%d, brake=%d", 
             mem_speed, spin_input_throttle, spin_input_brake);
}
