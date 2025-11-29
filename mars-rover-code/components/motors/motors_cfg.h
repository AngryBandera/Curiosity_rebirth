#pragma once

#include "hal/ledc_types.h"
#include <cstdint>
#include <cmath>

constexpr float PI = 3.14159265358979323846f;

namespace Servo {
    // Parameters for servo motors    
    constexpr uint16_t MIN_PULSE_US = 500;
    constexpr uint16_t MAX_PULSE_US = 2500;
    constexpr uint16_t PERIOD_US = 3333; // was 20000
    constexpr uint16_t  FREQ = 300; // it is maximum frequency for servos
    constexpr uint8_t RESOLUTION = 12;
    constexpr uint16_t MAX_DUTY = (1 << Servo::RESOLUTION);

    constexpr float DEGREE_TO_US = static_cast<float>(Servo::MAX_PULSE_US
            - Servo::MIN_PULSE_US) / 180.0f;

    constexpr uint16_t CENTER_DUTY = (Servo::MAX_DUTY *
            ((Servo::MIN_PULSE_US + Servo::MAX_PULSE_US) / 2)) / Servo::PERIOD_US;
}

namespace Cfg {
    // Wheel motor: speed is signed in internal units (e.g. -4095..+4095).
    // We map absolute speed -> PWM (0..4095). If you have a different internal max (e.g. 1000),
    // adjust `SCALE` accordingly or pass a max parameter.
    constexpr int16_t MOTOR_INTERNAL_MAX = 3000;
    constexpr float MOTOR_SCALE = 4095.0f / static_cast<float>(MOTOR_INTERNAL_MAX);

    // Default angles for steerable wheels
    constexpr float WHEEL_CENTER_ANGLE = 90.0f; // by default
    constexpr float WHEEL_MAX_DEVIATION = 80.0f;
    
    // Coordinates of wheels relative to mars rover center (in mm)
    constexpr int16_t FRONT_Y = 300;
    constexpr int16_t BACK_Y = -273;
    constexpr int16_t LEFT_X = -269;
    constexpr int16_t RIGHT_X = 269;

    // I am not sure whether we need these parameters
    // usually it is used for deviation from center that we neglect
    constexpr float ANGLE_DEVIATION = 0.5f;

    // dc motor acceleration (units per control loop tick)
    constexpr int16_t DC_ACCEL = 30;
    // servo speed (degrees per control loop tick)
    constexpr float SERVO_SPEED = 0.5f;
    
    // === НОВI ПАРАМЕТРИ ІНЕРЦІЇ ===
    // Час (в тактах) потрібний щоб зменшити швидкість на 1 одиницю при ковзанні
    // На швидкості 3000 будет гальмування протягом: 3000 * INERTIA_TICKS_PER_UNIT * 10ms
    // Приклад: INERTIA_TICKS_PER_UNIT = 2 => на 3000 потрібно 3000*2*10ms = 60 секунд
    constexpr float INERTIA_TICKS_PER_UNIT = 1.0f;  // Кількість тактів (×10мс) на одиницю швидкості
    constexpr float UINT_PER_INERTIA_TICKS = 1.0f / INERTIA_TICKS_PER_UNIT;

    // Max time for inertia effect (in ticks) to avoid too long sliding
    // 1000 тактів * 10 ms = 10 seconds
    constexpr uint16_t MAX_INERTIA_TICKS = 1000;
    
    // === PARAMETERS FOR SPINNING MODE ===
    // Angle for front wheels when spinning around center
    constexpr float SPIN_FRONT_ANGLE = 48.11847;      // degrees
    // Angle for back wheels when spinning around center
    constexpr float SPIN_BACK_ANGLE  = 45.42284;      // degrees
    
    // Max speed in spinning mode (internal units)
    constexpr int16_t SPIN_MAX_SPEED = 2000;
}

// === A4988 Stepper driver pin configuration ===
// Set these to your board GPIO numbers used for the A4988 driver.
// If left as -1 the pins are considered unconfigured.
namespace A4988 {
    constexpr int STEP_PIN = 33;                                                                                                                                                                                                                                                                                                                                                                   ; // a.k.a. CLK on some boards
    constexpr int DIR_PIN  = 19;
    constexpr int ENABLE_PIN = -1; // optional: driver enable (active low)
    // default pulse width for STEP in microseconds (private default)
    constexpr int STEP_PULSE_US = 2; // typical 1..5 us pulse
}
