#include "drive_system.h"
#include "esp_log.h"
#include "motors_cfg.h"
#include <memory>
#include <cmath>

inline uint32_t isqrt(uint32_t n) {
    if (n == 0) return 0;
    uint32_t x = n;
    uint32_t y = (x + 1) >> 1;
    while (y < x) {
        x = y;
        y = (x + n / x) >> 1;
    }
    return x;
}

static inline float approach_angle(float current, float target, float step) {
    float diff = target - current;
    if (fabsf(diff) <= step)
        return target;
    return current + copysignf(step, diff);
}


DriveSystem::DriveSystem(PCA9685Buffer* buffer)
    : buffer{buffer},
    right_back{
        buffer, 1, 0,
        "RightBackWheel",
        Cfg::BACK_Y, Cfg::RIGHT_X,
        12},
    right_middle{
        buffer, 2, 3, 
        "RightMiddleMotor",
        0, Cfg::RIGHT_X + 31},
    right_front{
        buffer, 5, 4,
        "RightFrontWheel", 
        Cfg::FRONT_Y, Cfg::RIGHT_X,
        13},

    left_back {
        buffer, 6, 7,
        "LeftBackWheel",
        Cfg::BACK_Y, Cfg::LEFT_X,
        14},
    left_middle {
        buffer, 9, 8,
        "LeftMiddleMotor",
        0, Cfg::LEFT_X - 31},
    left_front {
        buffer, 10, 11,
        "LeftFrontWheel",
        Cfg::FRONT_Y, Cfg::LEFT_X,
        15},

    mem_speed{0}, mem_angle{0.0f}, dest_speed{0}, dest_angle{0.0f}
{
    mutex = xSemaphoreCreateMutex();

    all_steerable_wheels[0] = &right_front;
    all_steerable_wheels[1] = &right_back;
    all_steerable_wheels[2] = &left_front;
    all_steerable_wheels[3] = &left_back;
    
    all_fixed_wheels[0] = &right_middle;
    all_fixed_wheels[1] = &left_middle;
}

DriveSystem* DriveSystem::create(gpio_num_t sda, gpio_num_t scl) {
    i2c_dev_t* i2c = new i2c_dev_t{};
    PCA9685Buffer* buffer = new PCA9685Buffer{i2c, sda, scl};
    return new DriveSystem{buffer};
}

void DriveSystem::print_angles() {
    ESP_LOGI(TAG, "rightBack: %.2f | rightFront: %.2f | leftBack: %.2f | lefftFront: %.2f",
            right_back.debug_angle, right_front.debug_angle, left_back.debug_angle, left_front.debug_angle);
}

void DriveSystem::move(int16_t speed, float rvr_angle) {
    if (fabsf(rvr_angle) <= 0.5f) {
        for (SteerableWheel* wheel : all_steerable_wheels) wheel->set_angle(0.0f);

        for (SteerableWheel* wheel : all_steerable_wheels) wheel->set_speed(speed);
        for (FixedWheel* wheel : all_fixed_wheels) wheel->set_speed(speed);
    } else {

        float alpha_rad = rvr_angle * PI / 180.0f;
        int32_t rvr_radius = static_cast<int32_t>(Cfg::FRONT_Y / tanf(alpha_rad));

        uint32_t steerable_radiuses[4];
        uint32_t fixed_radiuses[2];
        uint32_t maxR = 0;

        for (uint8_t i = 0; i < 4; i++) {
            auto* wheel = all_steerable_wheels[i];

            int8_t sign = (rvr_radius >= 0) ? 1 : -1;
            int32_t Y = (int32_t)wheel->get_Y();
            int32_t X_offset = rvr_radius - wheel->get_X();

            uint32_t inner_radius = isqrt((uint32_t)(Y*Y + X_offset*X_offset));
            float ratio = std::clamp((float)Y / (float)inner_radius, -1.0f, 1.0f);
            float angle = asinf(ratio) * 180.0f / PI * sign;

            wheel->set_angle(angle);
            steerable_radiuses[i] = inner_radius;
            if (inner_radius > maxR) maxR = inner_radius;
        }

        for (uint32_t i = 0; i < 2; i++) {
            auto* wheel = all_fixed_wheels[i];
            uint32_t inner_radius = abs(rvr_radius - wheel->get_X());
            fixed_radiuses[i] = inner_radius;
            maxR = std::max(maxR, inner_radius);
        }


        int sign = (speed >= 0) ? 1 : -1;
        uint32_t speed_abs = static_cast<uint32_t>(abs(speed));

        // fixed-point, shift to left 10
        uint32_t rot_speed_fp = (speed_abs << 10) / maxR;

        for (uint8_t i = 0; i < 4; i++) {
            all_steerable_wheels[i]->set_speed(static_cast<int16_t>
                ((rot_speed_fp * steerable_radiuses[i]) >> 10) * sign);
        }

        for (uint8_t i = 0; i < 2; i++) {
            all_fixed_wheels[i]->set_speed(static_cast<int16_t>
                ((rot_speed_fp * fixed_radiuses[i]) >> 10) * sign);
        }
    }

    buffer->flush();
}

void DriveSystem::set_angles_in_place(const bool go_to_spin) {
    for (auto* wheel : all_steerable_wheels) {
        wheel->set_angle(approach_angle(
            wheel->debug_angle,
            (go_to_spin) ? wheel->spin_target_angle : 0.0f,
            Cfg::SERVO_SPEED
        ));
    }
    buffer->flush();
}


void DriveSystem::rotate_in_place(int16_t speed) {
    uint32_t maxR = 403;

    float rot_speed = static_cast<float>(speed) /  static_cast<float>(maxR);

    right_back.set_speed(static_cast<int16_t>(-rot_speed * static_cast<float>(right_back.spin_radius)));
    right_middle.set_speed(static_cast<int16_t>(-rot_speed * static_cast<float>(right_middle.spin_radius)));
    right_front.set_speed(static_cast<int16_t>(-rot_speed * static_cast<float>(right_front.spin_radius)));

    left_back.set_speed(static_cast<int16_t>(rot_speed * static_cast<float>(left_back.spin_radius)));
    left_middle.set_speed(static_cast<int16_t>(rot_speed * static_cast<float>(left_middle.spin_radius)));
    left_front.set_speed(static_cast<int16_t>(rot_speed * static_cast<float>(left_front.spin_radius)));

    buffer->flush();
}



void DriveSystem::set(int16_t speed, float rvr_angle) {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
        dest_speed = speed;
        dest_angle = rvr_angle;
        xSemaphoreGive(mutex);
    }
}

void DriveSystem::set_speed(int16_t speed) {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
        dest_speed = speed;
        xSemaphoreGive(mutex);
    }
}

void DriveSystem::set_angle(float rvr_angle) {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
        dest_angle = rvr_angle;
        xSemaphoreGive(mutex);
    }
}

void DriveSystem::set_spin_input(int16_t throttle, int16_t brake) {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
        spin_input_throttle = throttle;
        spin_input_brake = brake;

        spin_active = (throttle > 5 || brake > 5);
        xSemaphoreGive(mutex);
    }
}


void DriveSystem::handle_idle() {
    mem_speed = 0;
    mem_angle = 0.0f;
}


void DriveSystem::handle_moving() {
    constexpr float MAX_ANGLE = 30.0f;

    float ratio = mem_angle / MAX_ANGLE;

    int16_t max_speed = (1.0f - std::pow(ratio, 3) / 2) * Cfg::MOTOR_INTERNAL_MAX;

    mem_speed += static_cast<int16_t>(std::clamp(dest_speed - mem_speed,
        (mem_speed > 0) ? -Cfg::DC_DECEL : -Cfg::DC_ACCEL,
        (mem_speed > 0) ? +Cfg::DC_ACCEL : +Cfg::DC_DECEL ));

    if (mem_speed > max_speed) mem_speed = max_speed;

    mem_angle = approach_angle(mem_angle, dest_angle, Cfg::SERVO_SPEED);
}


void DriveSystem::handle_stopping() {
    if (mem_speed != 0) {
        inertia_speed = mem_speed;
        mem_speed = 0;

        ESP_LOGD(TAG, "STOPPING: Inertia time = %u ticks (~%.1f sec)", 
                 inertia_ticks_remaining, inertia_ticks_remaining * 0.01f);
    }

    mem_angle = approach_angle(mem_angle, dest_angle, Cfg::SERVO_SPEED);
}

void DriveSystem::handle_angle_preparation() {
    bool all_reached = true;

    const float front_target = spin_active ? Cfg::SPIN_FRONT_ANGLE : 0.0f;
    const float back_target = spin_active ? Cfg::SPIN_BACK_ANGLE : 0.0f;

    all_reached &= (left_front.debug_angle == front_target);
    all_reached &= (right_front.debug_angle == -front_target);
    all_reached &= (right_back.debug_angle == back_target);
    all_reached &= (left_back.debug_angle == -back_target);

    if (all_reached) angle_achieved = true;
}

void DriveSystem::handle_spinning() {
    // Speed = throttle - brake
    int16_t spin_speed_input = spin_input_throttle - spin_input_brake;
    
    // Normalize to range [-SPIN_MAX_SPEED, SPIN_MAX_SPEED]
    float normalized = (float)spin_speed_input / 512.0f;  // 512 - max speed
    int16_t spin_speed = static_cast<int16_t>(normalized * (float)Cfg::SPIN_MAX_SPEED);

    mem_speed += static_cast<int16_t>(std::clamp(spin_speed - mem_speed,
        (mem_speed > 0) ? -Cfg::DC_DECEL : -Cfg::DC_ACCEL,
        (mem_speed > 0) ? +Cfg::DC_ACCEL : +Cfg::DC_DECEL));

    //ESP_LOGI(TAG, "SPINNING: speed=%d, throttle=%d, brake=%d", 
    //         mem_speed, spin_input_throttle, spin_input_brake);
}

void DriveSystem::apply_inertia() {
    // Apply inertia effect if we are in STOPPING
    if (inertia_speed == 0) return;

    inertia_speed += std::clamp<int16_t>(-inertia_speed, -Cfg::INNERT_DECEL, Cfg::INNERT_DECEL);
}

void DriveSystem::update_state() {
    //state_tick_counter++;

    switch (current_state) {
        case DriveState::IDLE:
            if (spin_active) {
                current_state = DriveState::ANGLE_PREPARATION;
                angle_achieved = false;
            } else if (std::abs(dest_speed) > 10 || fabsf(dest_angle) > 0.5f) {
                current_state = DriveState::MOVING;
            }
            break;

        case DriveState::MOVING:
            if (spin_active) {
                current_state = DriveState::STOPPING;
            } else if (std::abs(dest_speed) < 10 && fabsf(dest_angle) < 0.5f) {
                current_state = DriveState::IDLE;
            }
            break;

        case DriveState::STOPPING:
            if (inertia_ticks_remaining == 0) {
                current_state = DriveState::IDLE;
            } else if (!spin_active && abs(dest_speed) > 10) {
                current_state = DriveState::MOVING;
                inertia_ticks_remaining = 0;
            }
            break;

        case DriveState::SPINNING:
            static uint16_t idle_ticks = 0;

            if (spin_active) {
                idle_ticks = 0;
            } else if (++idle_ticks >= Cfg::SPIN_DEACTIVATE_TICKS) {
                idle_ticks = 0;
                current_state = DriveState::IDLE;
            }
            break;
        case DriveState::ANGLE_PREPARATION:
            if (angle_achieved) {
                current_state = (spin_active) ?
                        DriveState::SPINNING : DriveState::IDLE;
            }
            break;
    }
}

void DriveSystem::tick() {
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {

        update_state();

        print_state();
        
        switch (current_state) {
            case DriveState::IDLE:
                handle_idle();
                move(mem_speed, mem_angle);
                break;
            case DriveState::MOVING:
                handle_moving();
                move(mem_speed, mem_angle);
                break;
            case DriveState::STOPPING:
                handle_stopping();
                move(mem_speed, mem_angle);
                break;
            case DriveState::SPINNING:
                handle_spinning();
                rotate_in_place(mem_speed);
                break;
            case DriveState::ANGLE_PREPARATION:
                handle_angle_preparation();
                set_angles_in_place(spin_active);
                break;
        }

        // Apply innertian
        // it is used mainly in STOPPING state
        apply_inertia();

        xSemaphoreGive(mutex);
    }
}
            
void DriveSystem::print_state() {
    [[maybe_unused]] const char* state_names[] = {
        "IDLE", "MOVING", "STOPPING", "SPINNING", "ANGLE_PREPARATION"
    };
    ESP_LOGI(TAG, "State: %s | Speed: %d/%d | Angle: %.1f/%.1f | Inertia: %u | Spinning: %s",
            state_names[static_cast<int>(current_state)],
            mem_speed, dest_speed,
            mem_angle, dest_angle,
            inertia_ticks_remaining,
            spin_active ? "YES" : "NO");
}

DriveSystem::~DriveSystem() {
    if (mutex) vSemaphoreDelete(mutex);
    delete buffer;
}
