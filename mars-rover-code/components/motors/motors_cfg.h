#include "hal/ledc_types.h"
#include <cstdint>

namespace Servo {
    // Серво параметри    
    constexpr uint16_t MIN_PULSE_US = 500;
    constexpr uint16_t MAX_PULSE_US = 2500;
    constexpr uint16_t PERIOD_US = 20000;
    constexpr uint8_t  FREQ = 50;
    constexpr uint8_t RESOLUTION = 12;
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
}
