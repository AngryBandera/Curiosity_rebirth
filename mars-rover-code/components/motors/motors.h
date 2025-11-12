#include "i2cdev.h"
#include "motors_cfg.h"
#include <cstdint>
#include <sys/types.h>
#include "pca9685.h"

class WheelMotor {
protected:
    uint8_t pca1;
    uint8_t pca2;

    const char *TAG;

    const int16_t l;
    const int16_t d;

    uint32_t inner_radius;

public:

    WheelMotor(uint8_t pca1, uint8_t pca2,
            const char *TAG_init,
            int16_t l_new = 0, int16_t d_new = 0);

    /*
     * just makes it to move forward with no calculations
    */
    void move(int16_t speed, i2c_dev_t* pca9685);

    void update_radius(int32_t med_radius);

    uint32_t get_radius();

    void stop(i2c_dev_t* pca9685);
};



/*
 * extended WheelMotor class but includes servomotor manipulations
*/
class SteerableWheel: public WheelMotor {
private:
    uint8_t servo_pca;
    float current_angle{0.0f};

public:
    SteerableWheel(uint8_t pca1, uint8_t pca2,
            const char *TAG_init,
            int16_t l, int16_t d,
            uint8_t servo_pca);

    /*
     * angle - in interval [-45.00, 45.00]
     *  STRONGLY RECOMMENDED TO GIVE ONLY VALUES AS 1.0, 2.00, 30.0...
     */
    void rotate_on_relative_angle(i2c_dev_t* pca9685);

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
    i2c_dev_t* pca9685;
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
    DriveSystem(i2c_dev_t* pca9685);
    /*
     *angle - angle at which motors turn. It determines pwm duty cycle
     *here we call subclasses to recalculate value of inner classes
    */
    void rotate(float angle);

    /*
     * speed - determines pwm duty cycle
     * angle in bounds [-511, 511]
    */
    void move(int16_t speed);

    void print_angles();
};

