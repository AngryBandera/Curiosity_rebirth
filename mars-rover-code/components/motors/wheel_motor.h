#ifndef MOTORS_DRIVE_SYSTEM
#define MOTORS_DRIVE_SYSTEM

#include "i2cdev.h"
#include "motors_cfg.h"
#include "pca_buffer.h"
#include <cstdint>
#include <sys/types.h>

/*
Class for basic wheel motor (dc motor)

Is used for motors that do not rotate
and as a base class for SteerableWheel
*/
class WheelMotor {
protected:

    // PCA9685 channels for motor control
    uint8_t pca1;
    uint8_t pca2;

    // Logging tag for debugging
    const char *TAG;

    /* Geometry parameters
        l - distance from wheel to rotation center along Y axis
        d - distance from wheel to rotation center along X axis
    */
    const int16_t l;
    const int16_t d;

    //    Current inner radius of the wheel path
    uint32_t inner_radius{16000};
    uint16_t wheel_duty{0}; // Current PWM duty cycle for the wheel

public:

    WheelMotor(uint8_t pca1, uint8_t pca2,
            const char *TAG,
            int16_t l = 0, int16_t d = 0);


    //  Update wheel geometry based on radius of rover movement (rvr_radius)
    virtual void update_geometry(int32_t rvr_radius);

    // Writes speed value to PCA9685 buffer that will be flushed later
    virtual void update_buffer(int16_t speed, PCA9685Buffer* buffer);

    // Get current inner radius
    uint32_t get_radius();
};



/*
 * extended WheelMotor class but includes servomotor manipulations
 * for steering the wheel
*/
class SteerableWheel: public WheelMotor {
private:
    // PCA9685 channel for servo control
    uint8_t servo_pca;

    // Current angle of the wheel (in degrees)
    // 0 degrees means rotating straight forward
    float current_angle{0.0f};

    // Current PWM duty cycle for the servo
    uint16_t servo_duty{Servo::CENTER_DUTY};

public:
    // === PARAMETERS FOR SPINNING MODE ===
    uint16_t spin_servo_duty{Servo::CENTER_DUTY};  // PWM for servo in spinning mode
    float spin_target_angle{0.0f};                 // Target angle for spinning mode

    SteerableWheel(uint8_t pca1, uint8_t pca2,
            const char *TAG,
            int16_t l, int16_t d,
            uint8_t servo_pca);

    /* Override methods from WheelMotor
        Update wheel geometry based on rvr_radius and position of the wheel in space
    */
    void update_geometry(int32_t rvr_radius) override;

    void update_duty();

    void set_angle(float angle);

    // Writes speed and servo angle to PCA9685 buffer that will be flushed later
    void update_buffer(int16_t speed, PCA9685Buffer* buffer) override;

    // Get current angle of the wheel for logging/debugging
    float get_angle();
    
    // === METHODS FOR SPINNING MODE ===
    void calculate_spin_duty(float spin_angle);
};

#endif
