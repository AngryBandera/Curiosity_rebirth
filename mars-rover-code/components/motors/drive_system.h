#ifndef DRIVE_SYSTEM_H
#define DRIVE_SYSTEM_H

#include "wheel_motor.h"
#include "stepper_motor.h"


// Enum for states of the drive system
enum class DriveState {
    IDLE,           // Mars rover is stationary
    ACCELERATING,   // Accelerating to dest_speed
    MOVING,         // Moving at constant speed
    TURNING,        // Sharp turn with speed reduction
    STOPPING,       // Immidiate stop
    SPINNING        // Spinning in place
};


/* DriveSystem class
 This is a class where all the drive logic is implemented.

 It provides API interface for setting speed and turning angle of the rover.

 Functions:
    * Manages multiple WheelMotor and SteerableWheel instances
    * Implements a state machine for different driving modes

    * Sets speed and turning angle for the rover (Applies acceleration and deceleration)
    * Handles special spinning mode for turning in place
*/
class DriveSystem {
private:
    // Current radius of the turn (in mm)
    uint16_t radius{0};

    // Buffer for PCA9685 commands. Every tick esp32 flushes it to the device
    PCA9685Buffer* buffer;

    // Logging tag for debugging
    const char* TAG = "DriverSystem";

    // Wheel motor and steerable wheel instances
    SteerableWheel right_back;
    WheelMotor right_middle;
    SteerableWheel right_front;

    SteerableWheel left_back;
    WheelMotor left_middle;
    SteerableWheel left_front;

    // Arrays for easy access
    SteerableWheel* all_steerable_wheels[4];

    WheelMotor* all_wheels[6];

    // Stepper motor for camera pan control
    StepperMotor* camera_stepper;

    // === CURRENT AND DESTINATION SPEED/ANGLE ===
    // mem_speed - current speed that we are applying to motors (in real life that speed can be different due to inertia)
    // but it allows us to simulate acceleration/deceleration
    int16_t mem_speed;
    float mem_angle;

    // Desired speed and angle set by user
    int16_t dest_speed;
    float dest_angle;

    float actual_speed; // it is used only for inertia calculations
    // so don't be tricked by its name
    // in most of the cases mem_speed is an actual speed applied for motors

    // Internal method to directly move wheels without state machine
    void move_with_angle(int16_t speed, float angle);

    // === State machine ===
    DriveState current_state{DriveState::IDLE};
    DriveState previous_state{DriveState::IDLE};

    // Tick counter for the current state
    uint16_t state_tick_counter{0};
    
    // === Innercia parameters ===
    int16_t inertia_speed{0};
    uint16_t inertia_ticks_remaining{0};
    
    // === PARAMETERS FOR SPIN MODE ===
    // throttle and brake are buttons on joystick on front
    int16_t spin_input_throttle{0};    // Throttle value (0-512)
    int16_t spin_input_brake{0};       // Brake value (0-512)
    bool is_spinning{false};           // Whether we are in spinning mode
    
    // Methods for state machine handling
    void update_state();
    void handle_idle();
    void handle_accelerating();
    void handle_moving();
    void handle_decelerating();
    void handle_turning();
    void handle_stopping();
    void handle_spinning();
    
    // Helper methods for state transitions
    bool should_change_direction() const;
    bool should_start_turning() const;
    
    /* Method to apply inertia effect
    It is used in STOPPING and DECELERATING states for calculate whether rover stopped completely
    or still has some residual speed due to inertia
    */
    void apply_inertia();

public:

    DriveSystem(i2c_dev_t* pca9685);

    /*
     * speed - it's a speed of mars rover center, so using it DriverSystem class calculates speed for every motor
     * it lies in range [-4095, 4095] in internal units

     * angle - angle of rover turn in degrees
     *  - positive angle -> turn right
     *  - negative angle -> turn left
    */
    void set(int16_t speed, float angle);

    // Setters for speed and angle separately
    void set_speed(int16_t speed);
    void set_angle(float angle);
    
    /* === METHODS FOR SPIN MODE ===
        Set spin input from joystick buttons
        throttle - value from throttle button (0-512)
        brake - value from brake button (0-512)

        Speed in spin mode is calculated as (throttle - brake)
    */
    void set_spin_input(int16_t throttle, int16_t brake);
    void stop_spinning();

    /* === STEPPER MOTOR CONTROL ===
        Control camera pan stepper motor
        speed - normalized speed from -1.0 (full left) to 1.0 (full right)
    */
    void set_stepper_speed(float speed);
    
    /* === SERVO CONTROL ===
        Control camera vertical (tilt) servo
        angle - normalized angle from -1.0 (down) to 1.0 (up)
    */
    void set_servo_angle(float angle);
    
    StepperMotor* get_stepper_motor() { return camera_stepper; }

    
    // Methods to get internal parameters for debugging
    uint16_t get_inertia_ticks_remaining() const { return inertia_ticks_remaining; }
    bool get_is_spinning() const { return is_spinning; }
    
    /* IMPORTANT METHOD!
    * This method should be called periodically (e.g., every 10ms)
    * It updates the drive system state and applies necessary changes to motors
    
    * It allows the DriveSystem to manage acceleration and make movements smooth
    */
   void tick();
   
   // Immediately stop all motors, all PWM values to 0
   void stop();


   // Method to get current drive state for external monitoring
   DriveState get_current_state() const { return current_state; }

    // Debugging methods
    void print_angles();
    void print_state();

};

#endif
