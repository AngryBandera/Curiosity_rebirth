#include "stepper.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <stdlib.h>
#include <cmath>

static const char* TAG = "stepper";

// Local busy-wait microsecond delay using esp_timer to avoid depending on
// `esp_rom/ets.h`.
static inline void delay_us(uint32_t us) {
    int64_t start = esp_timer_get_time();
    while ((uint32_t)(esp_timer_get_time() - start) < us) {
        ;
    }
}

// Timer context for async step pulse
struct StepTimerCtx {
    esp_timer_handle_t timer;
    int pin;
};

static void IRAM_ATTR step_clear_cb(void* arg) {
    StepTimerCtx* ctx = (StepTimerCtx*)arg;
    if (ctx) {
        gpio_set_level((gpio_num_t)ctx->pin, 0);
        if (ctx->timer) {
            esp_timer_delete(ctx->timer);
        }
        free(ctx);
    }
}

Stepper::Stepper()
    : step_pin_(-1), dir_pin_(-1), enable_pin_(-1), steps_per_rev_(200), microstep_(16),
      pulse_us_default_(A4988::STEP_PULSE_US), position_steps_(0), target_steps_(0),
      target_rate_sps_(200), task_handle_(nullptr), lock_(nullptr), task_running_(false) {}

Stepper::~Stepper() {
    stop_task();
    if (lock_) {
        vSemaphoreDelete(lock_);
        lock_ = nullptr;
    }
}

esp_err_t Stepper::init(int step_pin, int dir_pin, int enable_pin, int steps_per_rev, int microstep, int default_pulse_us) {
    if (step_pin < 0) step_pin = A4988::STEP_PIN;
    if (dir_pin  < 0) dir_pin  = A4988::DIR_PIN;
    if (enable_pin < 0) enable_pin = A4988::ENABLE_PIN;

    if (step_pin < 0 || dir_pin < 0) {
        ESP_LOGW(TAG, "Stepper pins not configured (STEP=%d DIR=%d)", step_pin, dir_pin);
        return ESP_ERR_INVALID_ARG;
    }

    step_pin_ = step_pin;
    dir_pin_ = dir_pin;
    enable_pin_ = enable_pin;
    steps_per_rev_ = (steps_per_rev > 0) ? steps_per_rev : 200;
    microstep_ = (microstep > 0) ? microstep : 16;
    pulse_us_default_ = (default_pulse_us > 0) ? default_pulse_us : A4988::STEP_PULSE_US;

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << step_pin_);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << dir_pin_);
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)dir_pin_, 0);

    if (enable_pin_ >= 0) {
        io_conf.pin_bit_mask = (1ULL << enable_pin_);
        gpio_config(&io_conf);
        gpio_set_level((gpio_num_t)enable_pin_, 1); // disabled (A4988 active LOW)
    }

    gpio_set_level((gpio_num_t)step_pin_, 0);

    if (!lock_) {
        lock_ = xSemaphoreCreateMutex();
    }

    ESP_LOGI(TAG, "Stepper initialized STEP=%d DIR=%d EN=%d steps/rev=%d micro=%d",
             step_pin_, dir_pin_, enable_pin_, steps_per_rev_, microstep_);
    return ESP_OK;
}

esp_err_t Stepper::start_task(const char* name, int core, int prio, size_t stack) {
    if (task_running_) return ESP_OK;
    task_running_ = true;
    BaseType_t r = xTaskCreatePinnedToCore(&Stepper::task_entry_trampoline, name, stack, this, prio, &task_handle_, core);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "Failed to create stepper task");
        task_running_ = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void Stepper::stop_task() {
    if (!task_running_) return;
    task_running_ = false;
    if (task_handle_) {
        // ask the task to exit; it will delete itself
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }
}

void Stepper::set_direction(bool dir) {
    if (dir_pin_ < 0) {
        ESP_LOGW(TAG, "DIR pin not configured");
        return;
    }
    gpio_set_level((gpio_num_t)dir_pin_, dir ? 1 : 0);
}

void Stepper::do_step_pulse(int pulse_us) {
    if (step_pin_ < 0) {
        ESP_LOGW(TAG, "STEP pin not configured");
        return;
    }
    gpio_set_level((gpio_num_t)step_pin_, 1);
    if (pulse_us > 0) delay_us((uint32_t)pulse_us);
    gpio_set_level((gpio_num_t)step_pin_, 0);
    delay_us(1);
}

void Stepper::step_once(int pulse_us) {
    do_step_pulse(pulse_us > 0 ? pulse_us : pulse_us_default_);
}

void Stepper::step_once_async(int pulse_us) {
    if (step_pin_ < 0) {
        ESP_LOGW(TAG, "STEP pin not configured");
        return;
    }
    StepTimerCtx* ctx = (StepTimerCtx*)calloc(1, sizeof(StepTimerCtx));
    if (!ctx) {
        ESP_LOGE(TAG, "Failed to allocate timer context for async step");
        return;
    }
    ctx->pin = step_pin_;

    esp_timer_create_args_t args = {};
    args.callback = &step_clear_cb;
    args.arg = ctx;
    args.dispatch_method = ESP_TIMER_TASK;
    args.name = "step_clear";

    esp_err_t err = esp_timer_create(&args, &ctx->timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create esp_timer: %d", err);
        free(ctx);
        return;
    }

    gpio_set_level((gpio_num_t)step_pin_, 1);
    uint64_t t_us = (pulse_us > 0) ? (uint64_t)pulse_us : (uint64_t)pulse_us_default_;
    esp_timer_start_once(ctx->timer, t_us);
}

void Stepper::set_target_angle(float angle_deg, int steps_per_sec) {
    if (microstep_ <= 0 || steps_per_rev_ <= 0) return;
    // normalize angle to [0,360)
    float ang = fmodf(angle_deg, 360.0f);
    if (ang < 0) ang += 360.0f;
    int64_t steps = (int64_t)roundf(ang / 360.0f * (float)(steps_per_rev_ * microstep_));
    if (lock_) xSemaphoreTake(lock_, portMAX_DELAY);
    target_steps_ = steps;
    target_rate_sps_ = (steps_per_sec > 0) ? steps_per_sec : target_rate_sps_;
    if (lock_) xSemaphoreGive(lock_);
}

float Stepper::get_current_angle() {
    if (microstep_ <= 0 || steps_per_rev_ <= 0) return 0.0f;
    int64_t pos = position_steps_;
    // bring into [0, steps_per_rev*microstep)
    int64_t cycle = (int64_t)steps_per_rev_ * microstep_;
    int64_t p = ((pos % cycle) + cycle) % cycle;
    return (float)p * 360.0f / (float)cycle;
}


void Stepper::task_entry_trampoline(void* arg) {
    Stepper* s = (Stepper*)arg;
    if (s) s->task_entry();
    vTaskDelete(nullptr);
}

void Stepper::task_entry() {
    // simple control loop: move toward target_steps_ at target_rate_sps_
    while (task_running_) {
        int64_t target = target_steps_;
        int64_t pos = position_steps_;
        if (pos == target) {
            // nothing to do
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        int dir = (target > pos) ? 1 : -1;
        set_direction(dir > 0);

        // compute delay between steps from target_rate_sps_
        int steps_per_sec = target_rate_sps_ > 0 ? target_rate_sps_ : 200;
        int64_t interval_us = (steps_per_sec > 0) ? (1000000LL / steps_per_sec) : 5000;

        // one step
        do_step_pulse(pulse_us_default_);
        // update position
        if (dir > 0) position_steps_++;
        else position_steps_--;

        // small yield
        if (interval_us >= 1000) {
            vTaskDelay(pdMS_TO_TICKS((uint32_t)(interval_us / 1000)));
        } else {
            // busy-wait for sub-ms delays
            delay_us((uint32_t)interval_us);
        }
    }
}

