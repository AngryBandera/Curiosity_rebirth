#ifndef DRIVE_SYSTEM_H
#define DRIVE_SYSTEM_H

#include "wheels.h"
#include "freertos/semphr.h"

// Enum for states of the drive system
enum class DriveState {
    IDLE,           // Mars rover is stationary
    MOVING,         // Moving
    STOPPING,       // Immidiate stop
    ANGLE_PREPARATION, // prepares angles for spin mode in and out
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

    SemaphoreHandle_t mutex;

    // Wheel motor and steerable wheel instances
    SteerableWheel right_back;
    FixedWheel right_middle;
    SteerableWheel right_front;

    SteerableWheel left_back;
    FixedWheel left_middle;
    SteerableWheel left_front;

    // Arrays for easy access
    SteerableWheel* all_steerable_wheels[4];
    FixedWheel* all_fixed_wheels[2];

    // === CURRENT AND DESTINATION SPEED/ANGLE ===
    // mem_speed - current speed that we are applying to motors (in real life that speed can be different due to inertia)
    // but it allows us to simulate acceleration/deceleration
    int16_t mem_speed;
    float mem_angle;

    // Desired speed and angle set by user
    int16_t dest_speed;
    float dest_angle;

    // Internal method to directly move wheels without state machine
    void move(int16_t speed, float rvr_angle);
    void set_angles_in_place(bool go_to_spin);
    void rotate_in_place(int16_t speed);

    // === State machine ===
    DriveState current_state{DriveState::IDLE};

    // Tick counter for the current state
    // uint16_t state_tick_counter{0};

    // === Innercia parameters ===
    int16_t inertia_speed{0};
    uint16_t inertia_ticks_remaining{0};

    // === PARAMETERS FOR SPIN MODE ===
    // throttle and brake are buttons on joystick on front
    int16_t spin_input_throttle{0};    // Throttle value (0-512)
    int16_t spin_input_brake{0};       // Brake value (0-512)
    bool spin_active{false};           // Whether we are in spinning mode

    bool angle_achieved{true};
    
    // Methods for state machine handling
    void update_state();

    void handle_idle();
    void handle_moving();
    void handle_stopping();
    void handle_spinning();
    void handle_angle_preparation();
    
    /* Method to apply inertia effect
    It is used in STOPPING and DECELERATING states for calculate whether rover stopped completely
    or still has some residual speed due to inertia
    */
    void apply_inertia();

    DriveSystem(PCA9685Buffer* buffer);
    
public:

    static DriveSystem* create(gpio_num_t sda, gpio_num_t scl);

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
    
    /* IMPORTANT METHOD!
    * This method should be called periodically (e.g., every 10ms)
    * It updates the drive system state and applies necessary changes to motors
    
    * It allows the DriveSystem to manage acceleration and make movements smooth
    */
    void tick();
   
//================================================================
    // DEBUG SECTION
    // DON'T USE THESE VARIABLES, METHOD IN FINAL PRODUCT

    // Methods to get internal parameters for debugging
    uint16_t get_inertia_ticks_remaining() const { return inertia_ticks_remaining; }
    bool get_is_spinning() const { return spin_active; }

    // Method to get current drive state for external monitoring
    DriveState get_current_state() const { return current_state; }
   
    // Debugging methods
    void print_angles();
    void print_state();
//================================================================

    DriveSystem& operator=(const DriveSystem&) = delete;
    ~DriveSystem();
};

#endif
