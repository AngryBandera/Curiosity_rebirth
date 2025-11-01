#include "hal/ledc_types.h"
#include "soc/gpio_num.h"

class WheelMotor {
private:
    gpio_num_t pwm_pin;
    /*
     Move direction
     * 0 - forward
     * 1 - backward
    */
    gpio_num_t dir_pin;
    ledc_channel_t channel;

    const char *TAG;

    const uint16_t l;
    const uint16_t d;

    uint16_t inner_radius;

public:
    static ledc_timer_t shared_timer;

    WheelMotor(gpio_num_t pin_1, gpio_num_t pin_2,
            const char *TAG_init, ledc_channel_t channel_init,
            uint16_t l_init = 0, uint16_t d_init = 0);

    /*
     * just makes it to move forward with no calculations
    */
    void forward(uint8_t speed);
    void backward(uint8_t speed);

    /*
     * Class has calculated radius for this class, so
     * it moves forward with radius calculated
    */
    void forward_rot(uint16_t rot_speed);
    void backward_rot(uint16_t rot_speed);

    void change_radius(uint16_t radius);

    void stop();
};



/*
 * extended WheelMotor class but includes servomotor manipulations
*/
class SteerableWheel: public WheelMotor {
private:
    gpio_num_t servo_pin;
    ledc_channel_t servo_channel;
    uint16_t current_angle{0};

public:
    static ledc_timer_t servo_timer;

    SteerableWheel(gpio_num_t pin_1, gpio_num_t pin_2,
            const char *TAG_init, ledc_channel_t channel_init,
            uint16_t l_init, uint16_t d_init,
            gpio_num_t servo_pin_init, ledc_channel_t servo_channel_init);

    void set_angle(int16_t angle);
};



/*
 * to use this class you should first configure a timer and give it
 * as an argument
*/
class DriveSystem {
private:
    uint16_t radius{0};

    //WheelMotor LeftMiddleMotor{GPIO_NUM_4, GPIO_NUM_5, "LeftMiddleMotor", LEDC_CHANNEL_0};
    SteerableWheel LeftEdgeMotors[2] = {
        SteerableWheel{
            GPIO_NUM_4, GPIO_NUM_5,
            "LeftBackWheel", LEDC_CHANNEL_0,
            1, 2,
            GPIO_NUM_6, LEDC_CHANNEL_1
        },
        SteerableWheel{
            GPIO_NUM_4, GPIO_NUM_5,
            "LeftFrontWheel", LEDC_CHANNEL_2,
            1, 2,
            GPIO_NUM_6, LEDC_CHANNEL_3
        }
    };

    //WheelMotor RightMiddleMotor{GPIO_NUM_4, GPIO_NUM_5, "RightMiddleMotor", LEDC_CHANNEL_3};
    SteerableWheel RightEdgeMotors[2] = {
        SteerableWheel{
            GPIO_NUM_4, GPIO_NUM_5,
            "LeftBackWheel", LEDC_CHANNEL_4,
            1, 2,
            GPIO_NUM_6, LEDC_CHANNEL_5
        },
        SteerableWheel{
            GPIO_NUM_4, GPIO_NUM_5,
            "LeftFrontWheel", LEDC_CHANNEL_6,
            1, 2,
            GPIO_NUM_6, LEDC_CHANNEL_7
        }
    };

public:
    /*
     * timer - shared timer that wheel motors will use
    */
    DriveSystem(ledc_timer_t timer);
    /*
     *angle - angle at which motors turn. It determines pwm duty cycle
     *here we call subclasses to recalculate value of inner classes
    */
    void rotate(int16_t angle);

    /*
     * speed - determines pwm duty cycle
    */
    void forward(int16_t speed);
    void backward(int16_t speed);
};

