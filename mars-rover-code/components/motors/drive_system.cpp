#include "drive_system.h"
#include "esp_log.h"


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
        0, Cfg::RIGHT_X},
    right_front{
        8, 9,
        "RightFrontWheel", 
        Cfg::FRONT_Y, Cfg::RIGHT_X,
        1},

    left_back {
        11, 10,
        "LeftBackWheel",
        Cfg::BACK_Y, Cfg::LEFT_X,
        2},
    left_middle {
        12, 13,
        "LeftMiddleMotor",
        0, Cfg::LEFT_X},
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

void DriveSystem::move_with_angle(int16_t speed, float rvr_angle) {
    int32_t rvr_radius;
    if (fabsf(rvr_angle) <= 1.0f) {
        rvr_radius = 16001;
    } else {
        float alpha_rad = rvr_angle * PI / 180.0f;
        rvr_radius = static_cast<int32_t>(Cfg::FRONT_Y / tanf(alpha_rad));
    }

    for (uint8_t i = 0; i < 6; i++) all_wheels[i]->update_geometry(rvr_radius);

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


bool DriveSystem::should_change_direction() const {
    return (mem_speed > 0 && -100 > dest_speed) || 
           (0 > mem_speed && dest_speed > 100);
}

bool DriveSystem::should_start_turning() const {
    return (fabsf(dest_angle) > 10.0f);
}

void DriveSystem::handle_idle() {
    // Нічого не робимо, всі мотори вимкнені
    if (previous_state != DriveState::IDLE) {
        stop();
    }
}

void DriveSystem::handle_accelerating() {
    // just a simple acceleration towards dest_speed
    if (dest_speed > mem_speed) {
        if (mem_speed > 0) {
            mem_speed = std::min(dest_speed, 
                             static_cast<int16_t>(mem_speed + Cfg::DC_ACCEL));
        } else {
            mem_speed = std::min(dest_speed, 
                                 static_cast<int16_t>(mem_speed + Cfg::DC_DECEL));
        }
    }
    else if (dest_speed < mem_speed) {
        if (mem_speed > 0) {
            mem_speed = std::max(dest_speed,
                                 static_cast<int16_t>(mem_speed - Cfg::DC_DECEL));
        } else {
            mem_speed = std::max(dest_speed, 
                                 static_cast<int16_t>(mem_speed - Cfg::DC_ACCEL));
        }
    }
    
    // Smooth steering towards target angle
    if (fabsf(dest_angle - mem_angle) > 1.0f) {
        if (dest_angle > mem_angle) {
            mem_angle += std::max(Cfg::SERVO_SPEED, (dest_angle - mem_angle) * 0.2f);
        } else {
            mem_angle -= std::max(Cfg::SERVO_SPEED, (mem_angle - dest_angle) * 0.2f);
        }
    } else {
        mem_angle = dest_angle;
    }
}

void DriveSystem::handle_moving() {
    // Слідкуємо за невеликими змінами швидкості
    if (dest_speed > mem_speed) {
        mem_speed = std::min(dest_speed, 
                             static_cast<int16_t>(mem_speed + Cfg::DC_ACCEL));
    } else if (dest_speed < mem_speed) {
        mem_speed = std::max(dest_speed, 
                             static_cast<int16_t>(mem_speed - Cfg::DC_ACCEL));
    }
    
    // Обертання коліс без сильних ривків
    if (fabsf(dest_angle - mem_angle) > 1.0f) {
        if (dest_angle > mem_angle) {
            mem_angle += Cfg::SERVO_SPEED;
        } else {
            mem_angle -= Cfg::SERVO_SPEED;
        }
    }
}


void DriveSystem::handle_turning() {
    // Decrease speed when turning sharply
    float turn_percentage = fabsf(dest_angle) / Cfg::WHEEL_MAX_DEVIATION;
    int16_t max_turn_speed = static_cast<int16_t>(
        Cfg::MOTOR_INTERNAL_MAX * (1.0f - turn_percentage * 0.8f)  //TODO: tune factor
    );
    
    if (std::abs(mem_speed) > max_turn_speed) {
        if (mem_speed > 0) mem_speed = std::max(                      max_turn_speed,  static_cast<int16_t>(mem_speed - Cfg::DC_DECEL));
        else               mem_speed = std::min(static_cast<int16_t>(-max_turn_speed), static_cast<int16_t>(mem_speed + Cfg::DC_DECEL));
    } else if (std::abs(dest_speed) > max_turn_speed) {
        if (dest_speed > 0) mem_speed = std::min(static_cast<int16_t>(mem_speed + Cfg::DC_ACCEL),                       max_turn_speed );
        else                mem_speed = std::max(static_cast<int16_t>(mem_speed - Cfg::DC_ACCEL), static_cast<int16_t>(-max_turn_speed));
    }

    // Faster steering towards target angle
    constexpr float servo_rate = Cfg::SERVO_SPEED * 2.0f;
    if (dest_angle > mem_angle) {
        mem_angle += std::max(servo_rate, (dest_angle - mem_angle) * 0.2f);
    } else {
        mem_angle -= std::max(servo_rate, (mem_angle - dest_angle) * 0.2f);
    }
}

void DriveSystem::handle_stopping() {
    if (mem_speed != 0) {
        // Обчислюємо час ковзання залежно від швидкості
        // Формула: час = |швидкість| * INERTIA_TICKS_PER_UNIT
        uint16_t calculated_ticks = static_cast<uint16_t>(
            std::abs(mem_speed) * Cfg::INERTIA_TICKS_PER_UNIT
        );
        
        // Обмежуємо максимальним часом
        inertia_ticks_remaining = std::min(calculated_ticks, Cfg::MAX_INERTIA_TICKS);
        inertia_speed = mem_speed;
        
        mem_speed = 0;
        
        ESP_LOGD(TAG, "STOPPING: Inertia time = %u ticks (~%.1f sec)", 
                 inertia_ticks_remaining, inertia_ticks_remaining * 0.01f);
    }

    if (inertia_speed > 0) {
        inertia_speed -= Cfg::UINT_PER_INERTIA_TICKS;
        if (inertia_speed < 0) inertia_speed = 0;
    } else  {
        inertia_speed += Cfg::UINT_PER_INERTIA_TICKS;
        if (inertia_speed > 0) inertia_speed = 0;
    }
}

void DriveSystem::apply_inertia() {
    // Apply inertia effect if we are in STOPPING
    if (inertia_ticks_remaining > 0) {
        inertia_ticks_remaining--;
    }
}

void DriveSystem::update_state() {
    previous_state = current_state;
    state_tick_counter++;
    
    switch (current_state) {
        case DriveState::IDLE:
            if (is_spinning) {
                current_state = DriveState::STOPPING;
                state_tick_counter = 0;
            } else if (std::abs(dest_speed) > 10 || fabsf(dest_angle) > 0.5f) {
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
            } else if (mem_speed != dest_speed) {
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
                    break;
                } else if (dest_speed > actual_speed && dest_speed > 50) {
                    current_state = DriveState::TURNING;
                    state_tick_counter = 0;
                } else if (dest_speed < actual_speed && dest_speed < -50) {
                    current_state = DriveState::TURNING;
                    state_tick_counter = 0;
                }
            } else if (inertia_ticks_remaining == 0) {
                if (is_spinning) {
                    current_state = DriveState::SPINNING;
                    state_tick_counter = 0;
                } else if (!is_spinning) {
                    current_state = DriveState::IDLE;
                    state_tick_counter = 0;
                }
            }
            break;

        case DriveState::SPINNING:
            static uint16_t not_spinning_counter = 0;

            if (!is_spinning) {
                if (not_spinning_counter > Cfg::SPIN_DEACTIVATE_TICKS) {
                    // If spin inputs are released for enough time, stop spinning
                    is_spinning = false;
                    current_state = DriveState::IDLE;
                    state_tick_counter = 0;
                    mem_speed = 0;
                    mem_angle = 0.0f;
                } else {
                    not_spinning_counter++;
                }
            } else {
                not_spinning_counter = 0;
            }
            break;
    }
    
    last_dest_speed = dest_speed;
    last_dest_angle = dest_angle;
}

void DriveSystem::tick() {
    update_state();

    print_state();
    
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

    // Apply innertian
    // it is used mainly in STOPPING state
    apply_inertia();
    
    // And now apply actual speed depending on state
    if (current_state == DriveState::SPINNING) {
        for (uint8_t i = 0; i < 3; i++) {
            all_wheels[i]->update_buffer(-mem_speed, buffer);
        }
        for (uint8_t i = 3; i < 6; i++) {
            all_wheels[i]->update_buffer(mem_speed, buffer);
        }
        buffer->flush();
    } else {
        move_with_angle(mem_speed, mem_angle);
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


void DriveSystem::set_spin_input(int16_t throttle, int16_t brake) {
    spin_input_throttle = throttle;
    spin_input_brake = brake;
    
    if (throttle > 20 || brake > 20) {
        is_spinning = true;
    }
}

void DriveSystem::stop_spinning() {
    is_spinning = false;
    spin_input_throttle = 0;
    spin_input_brake = 0;
}

void DriveSystem::handle_spinning() {
    // Speed = throttle - brake
    int16_t spin_speed_input = spin_input_throttle - spin_input_brake;
    
    // Normalize to range [-SPIN_MAX_SPEED, SPIN_MAX_SPEED]
    float normalized = (float)spin_speed_input / 512.0f;  // 512 - max speed
    int16_t spin_speed = static_cast<int16_t>(normalized * Cfg::SPIN_MAX_SPEED);
    
    if (spin_speed > mem_speed) {
        mem_speed = std::min(spin_speed, 
                            static_cast<int16_t>(mem_speed + Cfg::DC_ACCEL));
    } else if (spin_speed < mem_speed) {
        mem_speed = std::max(spin_speed, 
                            static_cast<int16_t>(mem_speed - Cfg::DC_ACCEL));
    }
    
    // Front wheels
    if (fabsf(Cfg::SPIN_FRONT_ANGLE - right_front.spin_target_angle) > 0.5f) {
        left_front.set_angle(left_front.get_angle() * 0.97 + Cfg::SPIN_BACK_ANGLE * 0.03f);
        right_front.set_angle(right_front.get_angle() * 0.97 - Cfg::SPIN_BACK_ANGLE * 0.03f);
    } else {
        left_back.set_angle(Cfg::SPIN_FRONT_ANGLE);
        right_back.set_angle(-Cfg::SPIN_FRONT_ANGLE);
        ESP_LOGI(TAG, "TARGETED FRONT ANGLE ACHIEVED LEFT %.1f, RIGHT %.1f", left_front.get_angle(), right_front.get_angle());
    }
    
    // Back wheels
    if (fabsf(Cfg::SPIN_BACK_ANGLE - right_back.get_angle()) > 0.1f) {
        right_back.set_angle(right_back.get_angle() * 0.97 + Cfg::SPIN_BACK_ANGLE * 0.03f);
        left_back.set_angle(left_back.get_angle() * 0.97 - Cfg::SPIN_BACK_ANGLE * 0.03f);
    } else {
        right_back.set_angle(Cfg::SPIN_BACK_ANGLE);
        left_back.set_angle(-Cfg::SPIN_BACK_ANGLE);
        ESP_LOGI(TAG, "TARGETED BACK ANGLE ACHIEVED %.1f, %.1f", left_back.get_angle(), right_back.get_angle());
    }
    
    ESP_LOGI(TAG, "SPINNING: speed=%d, throttle=%d, brake=%d", 
             mem_speed, spin_input_throttle, spin_input_brake);
}
