#ifndef MOTORS_FILE_H
#define MOTORS_FILE_H

#include "pca_buffer.h"
#include <cstdint>

/*
Class for basic wheel (dc motor)

Is used for motors that do not rotate
and as a base class for SteerableWheel
*/
class FixedWheel {
protected:

    PCA9685Buffer* buffer;

    // PCA9685 channels for motor control
    uint8_t pca1;
    uint8_t pca2;

    // Logging tag for debugging
    const char *TAG;

    /* Geometry parameters
        l - distance from wheel to rotation center along Y axis
        d - distance from wheel to rotation center along X axis
    */
    const int32_t Y; //prev l
    const int32_t X; //prev d

public:

    uint32_t spin_radius;

    FixedWheel(PCA9685Buffer* buffer,
            uint8_t pca1, uint8_t pca2,
            const char *TAG,
            int32_t Y, int32_t X);


    int32_t get_Y() const;
    int32_t get_X() const;

    void set_speed(int16_t speed);

    //=============================================
    // DEBUG SECTION
    // DON'T USE THESE VARIABLES

    //    Current inner radius of the wheel path
    uint32_t inner_radius{16000};
    uint16_t debug_speed{0}; // Current PWM duty cycle for the wheel

    //=============================================
};



/*
 * extended WheelMotor class but includes servomotor manipulations
 * for steering the wheel
*/
class SteerableWheel: public FixedWheel {
private:
    // PCA9685 channel for servo control
    uint8_t servo_pca;

public:
    // === PARAMETERS FOR SPINNING STATE ===
    float spin_target_angle;                       // Target angle for spinning mode

    SteerableWheel(PCA9685Buffer* buffer,
            uint8_t pca1, uint8_t pca2,
            const char *TAG,
            int16_t l, int16_t d,
            uint8_t servo_pca);

    void set_angle(float angle);


//=============================================
    // DEBUG SECTION
    // DON'T USE THESE VARIABLES

    float debug_angle{0.0f};
//=============================================
};

#endif
