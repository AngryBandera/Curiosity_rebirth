#ifndef MOTORS_DRIVE_SYSTEM
#define MOTORS_DRIVE_SYSTEM

#include "i2cdev.h"
#include "motors_cfg.h"
#include "pca_buffer.h"
#include <cstdint>
#include <sys/types.h>



// Enum для машини станів
enum class DriveState {
    IDLE,           // Марсохід стоїть
    ACCELERATING,   // Розгін до dest_speed
    MOVING,         // Рухається зі сталою швидкістю
    DECELERATING,   // Гальмування
    TURNING,        // Разкий поворот зі зменшенням швидкості
    STOPPING,       // Критичне гальмування (зміна напрямку)
    SPINNING        // Обертання навколо своєї осі (новий стан)
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
    uint16_t servo_duty{Servo::CENTER_DUTY};
    
public:
    // === ПАРАМЕТРИ SPIN РЕЖИМУ ===
    uint16_t spin_servo_duty{Servo::CENTER_DUTY};  // PWM для сервомотора при обертанні
    float spin_target_angle{0.0f};                 // Цільовий кут при обертанні

    SteerableWheel(uint8_t pca1, uint8_t pca2,
            const char *TAG,
            int16_t l, int16_t d,
            uint8_t servo_pca);

    void update_geometry(int32_t med_radius) override;
    void update_buffer(int16_t speed, PCA9685Buffer* buffer) override;

    float get_angle();
    
    // === МЕТОДИ ДЛЯ SPIN РЕЖИМУ ===
    void calculate_spin_duty(float spin_angle);
    uint16_t get_spin_duty() const { return spin_servo_duty; }
};

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

    int16_t mem_speed;
    float mem_angle;

    int16_t dest_speed;
    float dest_angle;

    //inline void dc_move(int16_t speed);
    //inline void rotate(float rvr_angle);
    void actual_move(int16_t speed, float angle);

    // === МАШИНА СТАНІВ ===
    DriveState current_state{DriveState::IDLE};
    DriveState previous_state{DriveState::IDLE};
    
    int16_t last_dest_speed{0};
    float last_dest_angle{0.0f};
    uint16_t state_tick_counter{0};
    
    // === ПАРАМЕТРИ ІНЕРЦІЇ ===
    int16_t inertia_speed{0};
    uint16_t inertia_ticks_remaining{0};
    
    // === ПАРАМЕТРИ SPIN РЕЖИМУ ===
    int16_t spin_input_throttle{0};    // Значення дроселя (0-512)
    int16_t spin_input_brake{0};       // Значення гальма (0-512)
    bool is_spinning{false};           // Чи активний режим обертання
    
    // Методи для роботи зі станами
    void update_state();
    void handle_idle();
    void handle_accelerating();
    void handle_moving();
    void handle_decelerating();
    void handle_turning();
    void handle_stopping();
    void handle_spinning();          // Новий обробник для SPINNING
    
    bool should_change_direction() const;
    bool should_start_turning() const;
    
    void apply_inertia();

public:
    /*
     * timer - shared timer that wheel motors will use
    */
    DriveSystem(i2c_dev_t* pca9685);

    /*
     * speed - determines pwm duty cycle
     * angle in bounds [-4096, 4095]
    */
    void set(int16_t speed, float angle);

    void set_speed(int16_t speed);
    void set_angle(float angle);
    
    // === МЕТОДИ ДЛЯ SPIN РЕЖИМУ ===
    void set_spin_input(int16_t throttle, int16_t brake);
    void stop_spinning();

    /*
     * speed:
     *  - >= 0 => rotates clockwise
     *  - <  0 => rotates counter clockwise
     */
    void rotate(int16_t speed);

    DriveState get_current_state() const { return current_state; }

    // Новий метод для отримання інформації про інерцію
    uint16_t get_inertia_ticks_remaining() const { return inertia_ticks_remaining; }
    bool get_is_spinning() const { return is_spinning; }

    void tick();
    void stop();

    void print_angles();
    void print_state();

    bool is_moving();
};



#endif
