#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stepper_motor_encoder.h"

/**
 * @brief StepperMotor class for controlling a stepper motor with RMT
 * 
 * This class provides API for setting speed and direction of a stepper motor.
 * It uses ESP32's RMT peripheral for precise pulse generation with hardware acceleration.
 * The motor supports smooth acceleration and deceleration curves.
 */
class StepperMotor {
public:
    /**
     * @brief Configuration structure for StepperMotor
     */
    struct Config {
        gpio_num_t gpio_en;              // Enable pin
        gpio_num_t gpio_dir;             // Direction pin
        gpio_num_t gpio_step;            // Step pin
        uint8_t enable_level;            // Level to enable motor (0 or 1)
        uint32_t resolution_hz;          // RMT resolution in Hz (e.g., 1MHz)
        uint32_t min_speed_hz;           // Minimum speed in Hz (e.g., 500)
        uint32_t max_speed_hz;           // Maximum speed in Hz (e.g., 2000)
        uint32_t accel_sample_points;    // Number of sample points for acceleration curve
    };

    /**
     * @brief Direction enumeration
     */
    enum class Direction {
        CLOCKWISE = 0,
        COUNTER_CLOCKWISE = 1
    };

    /**
     * @brief Constructor
     * @param config Configuration structure
     */
    StepperMotor(const Config& config);

    /**
     * @brief Destructor
     */
    ~StepperMotor();

    /**
     * @brief Initialize the stepper motor hardware and RMT
     * @return ESP_OK on success
     */
    esp_err_t init();

    /**
     * @brief Set the motor speed (normalized -1.0 to 1.0)
     * @param speed Normalized speed: negative for one direction, positive for other
     *              0.0 = stopped, -1.0 = full speed CCW, 1.0 = full speed CW
     */
    void set_speed(float speed);

    /**
     * @brief Set the motor direction
     * @param dir Direction (CLOCKWISE or COUNTER_CLOCKWISE)
     */
    void set_direction(Direction dir);

    /**
     * @brief Get current speed setting
     * @return Current normalized speed (-1.0 to 1.0)
     */
    float get_speed() const { return current_speed; }

    /**
     * @brief Get current direction
     * @return Current direction
     */
    Direction get_direction() const { return current_direction; }

    /**
     * @brief Stop the motor immediately
     */
    void stop();

    /**
     * @brief Enable or disable the motor
     * @param enable true to enable, false to disable
     */
    void set_enabled(bool enable);

    /**
     * @brief Check if motor is enabled
     * @return true if enabled
     */
    bool is_enabled() const { return enabled; }

    /**
     * @brief Update function - call this periodically (e.g., every 10-20ms)
     * This function handles speed transitions, acceleration, and pulse generation
     */
    void update();

    /**
     * @brief Start the RTOS task for this motor
     * @param task_name Name of the task
     * @param priority Task priority
     * @param core_id Core ID to pin the task (0 or 1, or tskNO_AFFINITY)
     * @return ESP_OK on success
     */
    esp_err_t start_task(const char* task_name, UBaseType_t priority = 5, BaseType_t core_id = tskNO_AFFINITY);

    /**
     * @brief Stop the RTOS task
     */
    void stop_task();

private:
    // Configuration
    Config config;

    // Hardware handles
    rmt_channel_handle_t motor_chan;
    rmt_encoder_handle_t accel_encoder;
    rmt_encoder_handle_t uniform_encoder;
    rmt_encoder_handle_t decel_encoder;

    // State variables
    float current_speed;          // Current normalized speed (-1.0 to 1.0)
    float target_speed;           // Target speed to reach
    Direction current_direction;
    bool enabled;
    bool initialized;

    // Speed control
    uint32_t current_freq_hz;     // Current frequency in Hz
    uint32_t target_freq_hz;      // Target frequency in Hz
    
    // Acceleration parameters
    static constexpr uint32_t ACCEL_STEPS_PER_UPDATE = 10;  // Steps per update cycle
    static constexpr float SPEED_CHANGE_RATE = 0.1f;        // Rate of speed change per update

    // RTOS task
    TaskHandle_t task_handle;
    SemaphoreHandle_t mutex;
    bool task_running;

    // Private methods
    void update_internal();
    void send_pulses();
    uint32_t speed_to_frequency(float speed) const;
    static void task_function(void* param);

    const char* TAG = "StepperMotor";
};

#endif // STEPPER_MOTOR_H
