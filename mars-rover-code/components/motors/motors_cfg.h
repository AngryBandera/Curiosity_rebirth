#include "hal/ledc_types.h"
#include <cstdint>
#include <cmath>

namespace Servo {
    // Серво параметри    
    constexpr uint16_t MIN_PULSE_US = 500;
    constexpr uint16_t MAX_PULSE_US = 2500;
    constexpr uint16_t PERIOD_US = 3333; //20000
    constexpr uint16_t  FREQ = 300; //50
    constexpr uint8_t RESOLUTION = 12;
    constexpr uint16_t MAX_DUTY = (1 << Servo::RESOLUTION);

    constexpr float DEGREE_TO_US = static_cast<float>(Servo::MAX_PULSE_US
            - Servo::MIN_PULSE_US) / 180.0f;

    constexpr uint16_t CENTER_DUTY = (Servo::MAX_DUTY *
            ((Servo::MIN_PULSE_US + Servo::MAX_PULSE_US) / 2)) / Servo::PERIOD_US;
}

namespace Cfg {
    // Кути в градусах
    constexpr float WHEEL_CENTER_ANGLE = 90.0f;
    constexpr float WHEEL_MAX_DEVIATION = 80.0f;
    
    // Геометрія марсохода (в міліметрах)
    constexpr float WHEELBASE_MM = 573.0f;        // відстань між передніми і задніми колесами
    constexpr float TRACK_WIDTH_MM = 538.0f;      // відстань між лівими і правими колесами
    
    // Координати коліс відносно центру
    constexpr int16_t FRONT_Y = 300;       // мм від центру (вперед +)
    constexpr int16_t BACK_Y = -273;       // мм від центру (назад -)
    constexpr int16_t LEFT_X = -269;       // мм від центру (ліво -)
    constexpr int16_t RIGHT_X = 269;       // мм від центру (право +)

    constexpr float PI = 3.14159265358979323846f;

    constexpr float ANGLE_DEVIATION = 0.5f;

    constexpr int16_t DC_ACCEL = 30;
    constexpr float SERVO_ACCECL = 0.5f;
    
    // === НОВI ПАРАМЕТРИ ІНЕРЦІЇ ===
    // Час (в тактах) потрібний щоб зменшити швидкість на 1 одиницю при ковзанні
    // На швидкості 3000 будет гальмування протягом: 3000 * INERTIA_TICKS_PER_UNIT * 10ms
    // Приклад: INERTIA_TICKS_PER_UNIT = 2 => на 3000 потрібно 3000*2*10ms = 60 секунд
    constexpr float INERTIA_TICKS_PER_UNIT = 1.0f;  // Кількість тактів (×10мс) на одиницю швидкості
    constexpr float UINT_PER_INERTIA_TICKS = 1.0f / INERTIA_TICKS_PER_UNIT;

    // Максимальний час гальмування (в тактах) щоб машина не зависала
    // 1000 тактів = 10 секунд
    constexpr uint16_t MAX_INERTIA_TICKS = 1000;
    
    // === ПАРАМЕТРИ ОБЕРТАННЯ НАВКОЛО ОСІ ===
    // Кут для переднього колеса при обертанні навколо осі (внутрішнього радіуса)
    constexpr float SPIN_FRONT_ANGLE = 45.0f;      // градуси
    // Кут для задніго колеса при обертанні навколо осі (внутрішнього радіуса)
    constexpr float SPIN_BACK_ANGLE = -45.0f;      // градуси (протилежний напрям)
    
    // Максимальна швидкість обертання навколо осі
    constexpr int16_t SPIN_MAX_SPEED = 2000;
}


// Видали дублювання Servo namespace звідсіля!
// Тепер вони будуть в pca_buffer.h

// Видали:
// #define I2C_MASTER_SCL_IO    GPIO_NUM_22
// #define I2C_MASTER_SDA_IO    GPIO_NUM_21
// #define I2C_MASTER_FREQ_HZ   100000
// #define PCA9685_ADDR         PCA9685_ADDR_BASE

// Тепер вони в pca_buffer.h

