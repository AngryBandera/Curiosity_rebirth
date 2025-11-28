#include "stepper.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <stdlib.h>

// Local busy-wait microsecond delay using esp_timer. This avoids depending on
// `esp_rom/ets.h` which may be unavailable on some ESP-IDF versions.
static inline void delay_us(uint32_t us) {
    int64_t start = esp_timer_get_time();
    while ((uint32_t)(esp_timer_get_time() - start) < us) {
        ;
    }
}

static const char* TAG = "stepper";

static int s_step_pin = -1;
static int s_dir_pin = -1;
static int s_enable_pin = -1;

esp_err_t Stepper::init(int step_pin, int dir_pin, int enable_pin) {
    if (step_pin < 0) step_pin = A4988::STEP_PIN;
    if (dir_pin  < 0) dir_pin  = A4988::DIR_PIN;
    if (enable_pin < 0) enable_pin = A4988::ENABLE_PIN;

    if (step_pin < 0 || dir_pin < 0) {
        ESP_LOGW(TAG, "Stepper pins not configured (STEP=%d DIR=%d)", step_pin, dir_pin);
        return ESP_ERR_INVALID_ARG;
    }

    s_step_pin = step_pin;
    s_dir_pin = dir_pin;
    s_enable_pin = enable_pin;

    gpio_config_t io_conf = {};
    // configure step pin output
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << s_step_pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // configure dir pin output
    io_conf.pin_bit_mask = (1ULL << s_dir_pin);
    gpio_config(&io_conf);

    // optional enable pin
    if (s_enable_pin >= 0) {
        io_conf.pin_bit_mask = (1ULL << s_enable_pin);
        gpio_config(&io_conf);
        // default enabled (A4988 enable is active LOW) -> set high to disable
        gpio_set_level((gpio_num_t)s_enable_pin, 1);
    }

    // ensure step is low
    gpio_set_level((gpio_num_t)s_step_pin, 0);

    ESP_LOGI(TAG, "Stepper initialized STEP=%d DIR=%d EN=%d", s_step_pin, s_dir_pin, s_enable_pin);
    return ESP_OK;
}

void Stepper::set_direction(bool dir) {
    if (s_dir_pin < 0) {
        ESP_LOGW(TAG, "DIR pin not configured");
        return;
    }
    gpio_set_level((gpio_num_t)s_dir_pin, dir ? 1 : 0);
}

void Stepper::step_once(int pulse_us) {
    if (s_step_pin < 0) {
        ESP_LOGW(TAG, "STEP pin not configured");
        return;
    }

    // pulse: high -> wait -> low
    gpio_set_level((gpio_num_t)s_step_pin, 1);
    if (pulse_us > 0) {
        delay_us(pulse_us);
    }
    gpio_set_level((gpio_num_t)s_step_pin, 0);
    // small settle delay
    delay_us(1);
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
        // delete the timer and free context
        if (ctx->timer) {
            esp_timer_delete(ctx->timer);
        }
        free(ctx);
    }
}

void Stepper::step_once_async(int pulse_us) {
    if (s_step_pin < 0) {
        ESP_LOGW(TAG, "STEP pin not configured");
        return;
    }

    // allocate context for this pulse
    StepTimerCtx* ctx = (StepTimerCtx*)calloc(1, sizeof(StepTimerCtx));
    if (!ctx) {
        ESP_LOGE(TAG, "Failed to allocate timer context for async step");
        return;
    }
    ctx->pin = s_step_pin;

    esp_timer_create_args_t args = {};
    args.callback = &step_clear_cb;
    args.arg = ctx;
    args.dispatch_method = ESP_TIMER_TASK; // run in timer task
    args.name = "step_clear";

    esp_err_t err = esp_timer_create(&args, &ctx->timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create esp_timer: %d", err);
        free(ctx);
        return;
    }

    // set step high immediately and schedule clear
    gpio_set_level((gpio_num_t)s_step_pin, 1);
    uint64_t t_us = (pulse_us > 0) ? (uint64_t)pulse_us : (uint64_t)A4988::STEP_PULSE_US;
    esp_timer_start_once(ctx->timer, t_us);
}
