#ifndef MOTORS_DRIVE_SYSTEM
#define MOTORS_DRIVE_SYSTEM

#include "i2cdev.h"
#include "motors_cfg.h"
#include <cstdint>
#include <sys/types.h>
#include "pca9685.h"


class PCA9685Buffer {
private:
    i2c_dev_t* device;
    uint16_t buffer[16];  // Буфер для 16 каналів (0-4095)
    bool dirty;           // Чи є незбережені зміни
    const char* TAG = "PCA9685Buffer";
    
public:
    PCA9685Buffer(i2c_dev_t* pca9685);
    
    /*
     * Встановити PWM значення для каналу (не відправляє одразу!)
     * channel: 0-15
     * value: 0-4095
     */
    void set_channel_value(uint8_t channel, uint16_t value);
    
    /*
     * Отримати поточне значення з буфера
     */
    uint16_t get_channel_value(uint8_t channel);
    
    /*
     * Відправити всі накопичені зміни на PCA9685 однією транзакцією
     */
    void flush();
    
    /*
     * Встановити значення і одразу відправити
     */
    void set_channel_immediate(uint8_t channel, uint16_t value);
    
    /*
     * Чи є незбережені зміни
     */
    bool is_dirty() const;
    
    /*
     * Очистити буфер (встановити всі канали в 0)
     */
    void clear();
};



class WheelMotor {
protected:
    uint8_t pca1;
    uint8_t pca2;

    const char *TAG;

    const int16_t l;
    const int16_t d;

    uint32_t inner_radius{16000};
    uint16_t wheel_duty{0};

public:

    WheelMotor(uint8_t pca1, uint8_t pca2,
            const char *TAG,
            int16_t l = 0, int16_t d = 0);

    /*
     * just makes it to move forward with no calculations
    */
    virtual void update_geometry(int32_t med_radius);
    virtual void update_buffer(int16_t speed, PCA9685Buffer* buffer);

    uint32_t get_radius();
};



/*
 * extended WheelMotor class but includes servomotor manipulations
*/
class SteerableWheel: public WheelMotor {
private:
    uint8_t servo_pca;
    float current_angle{0.0f};
    uint16_t servo_duty{Servo::CENTER_DUTY}; //TODO: shouldn't be 0

public:
    SteerableWheel(uint8_t pca1, uint8_t pca2,
            const char *TAG,
            int16_t l, int16_t d,
            uint8_t servo_pca);

    /*
     * angle - in interval [-45.00, 45.00]
     *  STRONGLY RECOMMENDED TO GIVE ONLY VALUES AS 1.0, 2.00, 30.0...
     */
    void update_geometry(int32_t med_radius) override;
    void update_buffer(int16_t speed, PCA9685Buffer* buffer) override;

    float get_angle();
};



/*
 * to use this class you should first configure a timer and give it
 * as an argument
*/
class DriveSystem {
private:
    uint16_t radius{0};
    PCA9685Buffer* buffer;
    const char* TAG = "DriverSystem";

    SteerableWheel right_back;
    WheelMotor right_middle;
    SteerableWheel right_front;

    SteerableWheel left_back;
    WheelMotor left_middle;
    SteerableWheel left_front;

    SteerableWheel* all_steerable_wheels[4];

    WheelMotor* all_wheels[6];

    //inline void dc_move(int16_t speed);
    //inline void rotate(float rvr_angle);

public:
    /*
     * timer - shared timer that wheel motors will use
    */
    DriveSystem(i2c_dev_t* pca9685);

    /*
     * speed - determines pwm duty cycle
     * angle in bounds [-4096, 4095]
    */
    void move(int16_t speed, float angle);
    void stop();

    void print_angles();
};



#endif
