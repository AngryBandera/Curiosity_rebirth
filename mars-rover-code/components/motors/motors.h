#include "hal/ledc_types.h"
#include "soc/gpio_num.h"
#include <cstdint>

namespace Servo {
    // Серво параметри    
    constexpr uint16_t MIN_PULSE_US = 1000;
    constexpr uint16_t MAX_PULSE_US = 2000;
    constexpr uint16_t PERIOD_US = 20000;
    constexpr uint8_t  FREQ = 50;
    constexpr ledc_timer_bit_t RESOLUTION = LEDC_TIMER_14_BIT;
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


class WheelMotor {
protected:
    gpio_num_t pwm_pin;
    /*
     Move direction
     * 0 - forward
     * 1 - backward
    */
    gpio_num_t dir_pin;
    ledc_channel_t channel;

    const char *TAG;

    const int16_t l;
    const int16_t d;

    uint32_t inner_radius;

public:
    static ledc_timer_t shared_timer;

    WheelMotor(gpio_num_t pin_1, gpio_num_t pin_2,
            const char *TAG_init, ledc_channel_t channel_init,
            int16_t l_init = 0, int16_t d_init = 0);

    /*
     * just makes it to move forward with no calculations
    */
    void forward(uint8_t speed);
    void backward(uint8_t speed);

    /*
     * Class has calculated radius for this class, so
     * it moves forward with radius calculated
    */
    void forward_rot(uint8_t speed, int32_t med_radius);
    void backward_rot(uint8_t speed, int32_t med_radius);

    void update_radius(int32_t med_radius);
    uint32_t get_radius();

    void stop();
};



/*
 * extended WheelMotor class but includes servomotor manipulations
*/
class SteerableWheel: public WheelMotor {
private:
    gpio_num_t servo_pin;
    ledc_channel_t servo_channel;
    float current_angle{0.0f};

public:
    static ledc_timer_t servo_timer;

    SteerableWheel(gpio_num_t pin_1, gpio_num_t pin_2,
            const char *TAG_init, ledc_channel_t channel_init,
            int16_t l_init, int16_t d_init,
            gpio_num_t servo_pin_init, ledc_channel_t servo_channel_init);

    /*
     * angle - in interval [-45.00, 45.00]
     *  STRONGLY RECOMMENDED TO GIVE ONLY VALUES AS 1.0, 2.00, 30.0...
     */
    void rotate_on_relative_angle();

    void update_radius(int32_t med_radius) = delete;

    //TODO: keep in mind that when alpha does to 0, radius goes to infinity
    //min(alpha)=100, max(med_radius) = 17186
    void update_radius_and_angle(int32_t med_radius, float angle);

    float get_angle();
};



/*
 * to use this class you should first configure a timer and give it
 * as an argument
*/
class DriveSystem {
private:
    uint16_t radius{0};
    float prev_angle{0.0f};
    const char* TAG = "DriverSystem";

    SteerableWheel right_back;
    //WheelMotor right_middle;
    SteerableWheel right_front;

    SteerableWheel left_back;
    //WheelMotor left_middle;
    SteerableWheel left_front;

    SteerableWheel* all_steerable_wheels[4];

public:
    /*
     * timer - shared timer that wheel motors will use
    */
    DriveSystem(ledc_timer_t timer, ledc_timer_t servo_timer);
    /*
     *angle - angle at which motors turn. It determines pwm duty cycle
     *here we call subclasses to recalculate value of inner classes
    */
    void rotate(float angle);

    /*
     * speed - determines pwm duty cycle
    */
    void forward(int8_t speed);
    void backward(int8_t speed);

    void print_angles();
};

