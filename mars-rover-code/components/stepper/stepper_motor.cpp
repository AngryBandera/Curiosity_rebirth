#include "stepper_motor.h"
#include <cmath>
#include <algorithm>

StepperMotor::StepperMotor(const Config& config)
    : config(config)
    , motor_chan(nullptr)
    , accel_encoder(nullptr)
    , uniform_encoder(nullptr)
    , decel_encoder(nullptr)
    , current_speed(0.0f)
    , target_speed(0.0f)
    , current_direction(Direction::CLOCKWISE)
    , enabled(false)
    , initialized(false)
    , current_freq_hz(config.min_speed_hz)
    , target_freq_hz(config.min_speed_hz)
    , task_handle(nullptr)
    , mutex(nullptr)
    , task_running(false)
{
}

StepperMotor::~StepperMotor()
{
    stop_task();
    
    if (initialized) {
        stop();
        set_enabled(false);
        
        // Disable RMT channel
        if (motor_chan) {
            rmt_disable(motor_chan);
        }
        
        // Delete encoders
        if (accel_encoder) {
            rmt_del_encoder(accel_encoder);
        }
        if (uniform_encoder) {
            rmt_del_encoder(uniform_encoder);
        }
        if (decel_encoder) {
            rmt_del_encoder(decel_encoder);
        }
        
        // Delete RMT channel
        if (motor_chan) {
            rmt_del_channel(motor_chan);
        }
    }
    
    if (mutex) {
        vSemaphoreDelete(mutex);
    }
}

esp_err_t StepperMotor::init()
{
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing stepper motor");

    // Create mutex
    mutex = xSemaphoreCreateMutex();
    if (!mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize EN + DIR GPIO
    gpio_config_t en_dir_gpio_config = {
        .pin_bit_mask = (1ULL << config.gpio_en) | (1ULL << config.gpio_dir),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config));

    // Set initial direction
    gpio_set_level(config.gpio_dir, static_cast<uint32_t>(current_direction));
    
    // Disable motor initially
    gpio_set_level(config.gpio_en, !config.enable_level);

    // Create RMT TX channel
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = config.gpio_step,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = config.resolution_hz,
        .mem_block_symbols = 64,
        .trans_queue_depth = 100,
        .intr_priority = 0,
        .flags = {
            .invert_out = false,
            .with_dma = false,
            .io_loop_back = false,
            .io_od_mode = false,
        }
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor_chan));

    // Create acceleration encoder
    stepper_motor_curve_encoder_config_t accel_encoder_config = {
        .resolution = config.resolution_hz,
        .sample_points = config.accel_sample_points,
        .start_freq_hz = config.min_speed_hz,
        .end_freq_hz = config.max_speed_hz,
    };
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config, &accel_encoder));

    // Create uniform speed encoder
    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution = config.resolution_hz,
    };
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_encoder));

    // Create deceleration encoder
    stepper_motor_curve_encoder_config_t decel_encoder_config = {
        .resolution = config.resolution_hz,
        .sample_points = config.accel_sample_points,
        .start_freq_hz = config.max_speed_hz,
        .end_freq_hz = config.min_speed_hz,
    };
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config, &decel_encoder));

    // Enable RMT channel
    ESP_ERROR_CHECK(rmt_enable(motor_chan));

    initialized = true;
    ESP_LOGI(TAG, "Stepper motor initialized successfully");
    
    return ESP_OK;
}

void StepperMotor::set_speed(float speed)
{
    // Clamp speed to [-1.0, 1.0]
    speed = std::max(-1.0f, std::min(1.0f, speed));
    
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
        target_speed = speed;
        
        // Calculate target frequency based on absolute speed
        target_freq_hz = speed_to_frequency(std::abs(speed));
        
        // Note: Direction change is now handled in update_internal() based on current_speed
        // to prevent direction flip during deceleration through zero
        
        xSemaphoreGive(mutex);
    }
}

void StepperMotor::set_direction(Direction dir)
{
    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
        if (dir != current_direction) {
            current_direction = dir;
            gpio_set_level(config.gpio_dir, static_cast<uint32_t>(current_direction));
        }
        xSemaphoreGive(mutex);
    }
}

void StepperMotor::stop()
{
    set_speed(0.0f);
}

void StepperMotor::set_enabled(bool enable)
{
    enabled = enable;
    gpio_set_level(config.gpio_en, enable ? config.enable_level : !config.enable_level);
    
    if (enable) {
        ESP_LOGI(TAG, "Motor enabled");
    } else {
        ESP_LOGI(TAG, "Motor disabled");
        stop();
    }
}

void StepperMotor::update()
{
    if (!initialized || !enabled) {
        return;
    }

    if (xSemaphoreTake(mutex, portMAX_DELAY)) {
        update_internal();
        xSemaphoreGive(mutex);
    }
}

void StepperMotor::update_internal()
{
    // Smoothly transition current speed towards target speed
    if (std::abs(current_speed - target_speed) > 0.01f) {
        float speed_diff = target_speed - current_speed;
        float speed_change = std::copysign(
            std::min(SPEED_CHANGE_RATE, std::abs(speed_diff)),
            speed_diff
        );
        current_speed += speed_change;
    } else {
        current_speed = target_speed;
    }

    // Update direction based on current_speed (not target), and only when motor is actually moving
    // This prevents direction flip when decelerating through zero
    if (std::abs(current_speed) > 0.01f) {
        Direction new_dir = (current_speed >= 0) ? Direction::CLOCKWISE : Direction::COUNTER_CLOCKWISE;
        if (new_dir != current_direction) {
            current_direction = new_dir;
            gpio_set_level(config.gpio_dir, static_cast<uint32_t>(current_direction));
        }
    }

    // Smoothly transition current frequency towards target frequency
    if (current_freq_hz != target_freq_hz) {
        int32_t freq_diff = target_freq_hz - current_freq_hz;
        int32_t freq_step = (freq_diff > 0) ? 
            std::min((int32_t)20, freq_diff) : 
            std::max((int32_t)-20, freq_diff);
        current_freq_hz += freq_step;
    }

    // Send pulses if motor is moving
    if (std::abs(current_speed) > 0.01f) {
        send_pulses();
    }
}

void StepperMotor::send_pulses()
{
    // Prepare transmission configuration
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = true,  // Non-blocking to keep filling the queue
        }
    };

    // Send continuous pulses at current frequency
    // Try to add a few transmissions to keep queue filled, but stop when full
    // Reduced from 20 to 5 to avoid flooding the queue
    for (int i = 0; i < 5; i++) {
        esp_err_t err = rmt_transmit(motor_chan, uniform_encoder, &current_freq_hz, 
                                      sizeof(current_freq_hz), &tx_config);
        
        // If queue is full, stop trying - this is normal during continuous operation
        if (err == ESP_ERR_INVALID_STATE) {
            break;
        }
        
        // Silently ignore errors - they don't affect operation
        if (err != ESP_OK) {
            break;
        }
    }
}

uint32_t StepperMotor::speed_to_frequency(float speed) const
{
    if (speed < 0.01f) {
        return config.min_speed_hz;
    }
    
    // Linear mapping from speed [0, 1] to frequency [min_speed_hz, max_speed_hz]
    uint32_t freq_range = config.max_speed_hz - config.min_speed_hz;
    uint32_t freq = config.min_speed_hz + static_cast<uint32_t>(speed * freq_range);
    
    return std::max(config.min_speed_hz, std::min(config.max_speed_hz, freq));
}

esp_err_t StepperMotor::start_task(const char* task_name, UBaseType_t priority, BaseType_t core_id)
{
    if (task_running) {
        ESP_LOGW(TAG, "Task already running");
        return ESP_OK;
    }

    if (!initialized) {
        ESP_LOGE(TAG, "Cannot start task: motor not initialized");
        return ESP_FAIL;
    }

    task_running = true;

    BaseType_t result;
    if (core_id == tskNO_AFFINITY) {
        result = xTaskCreate(task_function, task_name, 4096, this, priority, &task_handle);
    } else {
        result = xTaskCreatePinnedToCore(task_function, task_name, 4096, this, 
                                         priority, &task_handle, core_id);
    }

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        task_running = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Task started: %s", task_name);
    return ESP_OK;
}

void StepperMotor::stop_task()
{
    if (task_running && task_handle) {
        task_running = false;
        vTaskDelete(task_handle);
        task_handle = nullptr;
        ESP_LOGI(TAG, "Task stopped");
    }
}

void StepperMotor::task_function(void* param)
{
    StepperMotor* motor = static_cast<StepperMotor*>(param);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms update rate (100Hz) for smooth motion

    ESP_LOGI(motor->TAG, "Task running");

    while (motor->task_running) {
        motor->update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    ESP_LOGI(motor->TAG, "Task exiting");
    vTaskDelete(NULL);
}
