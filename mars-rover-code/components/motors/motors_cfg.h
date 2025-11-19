#include "hal/ledc_types.h"
#include <cstdint>

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
}



// I2C and PCA9685 definitions
#define I2C_MASTER_SCL_IO    GPIO_NUM_22    // GPIO for SCL
#define I2C_MASTER_SDA_IO    GPIO_NUM_21    // GPIO for SDA
#define I2C_MASTER_FREQ_HZ   100000
#define PCA9685_ADDR         PCA9685_ADDR_BASE  // 0x40

