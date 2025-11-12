#include "hal/ledc_types.h"
#include "soc/gpio_num.h"
#include "motors_cfg.h"
#include <cstdint>

class WheelMotor {
protected:
    gpio_num_t pin1;
    gpio_num_t pin2;

    ledc_channel_t channel1;
    ledc_channel_t channel2;

    const char *TAG;

    const int16_t l;
    const int16_t d;

    uint32_t inner_radius;

public:
    static ledc_timer_t shared_timer;

    WheelMotor(gpio_num_t pin_1, gpio_num_t pin_2,
            const char *TAG_init,
            ledc_channel_t channel_1, ledc_channel_t channel_2,
            int16_t l_new = 0, int16_t d_new = 0);

    /*
     * just makes it to move forward with no calculations
    */
    void forward(uint8_t speed);
    void backward(uint8_t speed);

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
            const char *TAG_init,
            ledc_channel_t channel_1, ledc_channel_t channel_2,
            int16_t l_init, int16_t d_init,
            gpio_num_t servo_pin_init, ledc_channel_t servo_channel_init);

    /*
     * angle - in interval [-45.00, 45.00]
     *  STRONGLY RECOMMENDED TO GIVE ONLY VALUES AS 1.0, 2.00, 30.0...
     */
    void rotate_on_relative_angle();

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
    WheelMotor right_middle;
    SteerableWheel right_front;

    SteerableWheel left_back;
    WheelMotor left_middle;
    SteerableWheel left_front;

    SteerableWheel* all_steerable_wheels[4];

    WheelMotor* all_wheels[6];

public:
    /*
     * timer - shared timer that wheel motors will use
    */
    DriveSystem(ledc_timer_t dc_timer, ledc_timer_t servo_timer);
    /*
     *angle - angle at which motors turn. It determines pwm duty cycle
     *here we call subclasses to recalculate value of inner classes
    */
    void rotate(float angle);

    /*
     * speed - determines pwm duty cycle
    */
    void forward(uint8_t speed);
    void backward(uint8_t speed);

    void print_angles();
};

