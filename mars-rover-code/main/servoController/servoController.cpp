#include "ServoController.h"
#include "servoInterface.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include <stdint.h>
#include <string.h>
#include "driver/ledc.h"

#define FRONT_LEFT_SERVO_PIN  25
#define FRONT_RIGHT_SERVO_PIN 26
#define BACK_LEFT_SERVO_PIN   27
#define BACK_RIGHT_SERVO_PIN  14

#define SERVO_LEFT_ANGLE   0
#define SERVO_RIGHT_ANGLE  180
#define SERVO_CENTER_ANGLE 90

static const char* MOTOR_TAG = "MOTORS";

static ServoController frontLeft(FRONT_LEFT_SERVO_PIN);
static ServoController frontRight(FRONT_RIGHT_SERVO_PIN);
static ServoController backLeft(BACK_LEFT_SERVO_PIN);
static ServoController backRight(BACK_RIGHT_SERVO_PIN);

extern "C" void servos_init(void) {
    ServoController::init_shared_timer(); 
    frontLeft.setAngle(SERVO_CENTER_ANGLE);
    frontRight.setAngle(SERVO_CENTER_ANGLE);
    backLeft.setAngle(SERVO_CENTER_ANGLE);
    backRight.setAngle(SERVO_CENTER_ANGLE);

    ESP_LOGI(MOTOR_TAG, "Servos initialized and centered.");
}

extern "C" void frontServoLeft(void) {
    int new_angle = frontLeft.getCurrentAngle() - 10;
    frontLeft.setAngle(new_angle);
    frontRight.setAngle(new_angle);
    ESP_LOGI(MOTOR_TAG, "Front servos turned left");
}
extern "C" void frontServoRight(void) {
    int new_angle = frontLeft.getCurrentAngle() + 10;
    frontLeft.setAngle(new_angle);
    frontRight.setAngle(new_angle);
    ESP_LOGI(MOTOR_TAG, "Front servos turned right");
}
extern "C" void backServoLeft(void) {
    int new_angle = backLeft.getCurrentAngle() - 10;
    backLeft.setAngle(new_angle);
    backRight.setAngle(new_angle);
    ESP_LOGI(MOTOR_TAG, "Back servos turned left");
}
extern "C" void backServoRight(void) {
    int new_angle = backLeft.getCurrentAngle() + 10;
    backLeft.setAngle(new_angle);
    backRight.setAngle(new_angle);
    ESP_LOGI(MOTOR_TAG, "Back servos turned right");
}



const char* ServoController::TAG = "ServoController";
mcpwm_timer_handle_t ServoController::timer = NULL;
mcpwm_oper_handle_t ServoController::shared_oper = NULL;
bool ServoController::ledc_timer_inited = false;
int ServoController::next_ledc_channel = 0;
ledc_timer_t ServoController::ledc_timer_num = (ledc_timer_t)0;

void ServoController::init_shared_timer(int group_id) {
    if (timer) {
        ESP_LOGW(TAG, "Timer already initialized");
        return;
    }

    ESP_LOGI(TAG, "Initializing shared MCPWM timer");

    mcpwm_timer_config_t timer_config = {0};
    timer_config.group_id = group_id;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = 1000000; // 1MHz resolution (1 tick = 1us)
    timer_config.period_ticks = 20000;    // 20,000us period = 20ms = 50Hz
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;

    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

ServoController::ServoController(int gpio_pin, int min_pulse_us, int max_pulse_us)
    : comparator(NULL), generator(NULL), pin(gpio_pin), current_angle(-1), min_pulse(min_pulse_us), max_pulse(max_pulse_us), use_ledc(false), ledc_channel(-1) {

        ESP_LOGI(TAG, "ServoController constructed for GPIO %d (min=%d max=%d)", pin, min_pulse, max_pulse);
        // Do not require timer at construction time; support lazy init so static objects can be used.
}

ServoController::~ServoController() {
    ESP_LOGI(TAG, "De-initializing servo on GPIO %d", pin);
    if (comparator) {
        ESP_ERROR_CHECK(mcpwm_del_comparator(comparator));
        comparator = NULL;
    }
    if (generator) {
        ESP_ERROR_CHECK(mcpwm_del_generator(generator));
        generator = NULL;
    }
}

uint32_t ServoController::angleToCompare(int angle) const {
    // Map angle [0..180] to compare ticks (min_pulse..max_pulse), matching example mapping.
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    uint32_t range_pulse = (uint32_t)(max_pulse - min_pulse);
    return (uint32_t)((angle * range_pulse) / 180) + (uint32_t)min_pulse;
}

void ServoController::ensureOperatorInitialized() {
    if (comparator && generator) return; // already ready

    if (timer == NULL) {
        ESP_LOGE(TAG, "Timer not initialized! Call ServoController::init_shared_timer() first.");
        return;
    }

    ESP_LOGI(TAG, "Initializing MCPWM operator/comparator/generator for GPIO %d", pin);

    // create shared operator only once per group
    if (shared_oper == NULL) {
        mcpwm_operator_config_t operator_config = {0};
        operator_config.group_id = 0;
        esp_err_t err = mcpwm_new_operator(&operator_config, &shared_oper);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create shared operator (err=0x%x). No free operators in group?", err);
            shared_oper = NULL;
            return; // cannot proceed
        }
        // connect operator to shared timer
        err = mcpwm_operator_connect_timer(shared_oper, timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to connect shared operator to timer (err=0x%x)", err);
            // cleanup operator
            mcpwm_del_operator(shared_oper);
            shared_oper = NULL;
            return;
        }
    }

    mcpwm_comparator_config_t comparator_config = {0};
    comparator_config.flags.update_cmp_on_tez = true;
    esp_err_t err = mcpwm_new_comparator(shared_oper, &comparator_config, &comparator);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create comparator for GPIO %d (err=0x%x)", pin, err);
        comparator = NULL;
        ESP_LOGI(TAG, "Falling back to LEDC for GPIO %d", pin);
        // try LEDC fallback
        initLEDC();
        return;
    }

    mcpwm_generator_config_t generator_config = {0};
    generator_config.gen_gpio_num = pin;
    err = mcpwm_new_generator(shared_oper, &generator_config, &generator);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create generator for GPIO %d (err=0x%x)", pin, err);
        generator = NULL;
        if (comparator) {
            mcpwm_del_comparator(comparator);
            comparator = NULL;
        }
        ESP_LOGI(TAG, "Falling back to LEDC for GPIO %d", pin);
        // try LEDC fallback
        initLEDC();
        return;
    }

    // set initial center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angleToCompare(90)));

    // generator actions: high at timer empty, low at compare
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                 MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    // set initial angle
    current_angle = 90;
}

void ServoController::initLEDC() {
    // configure shared LEDC timer once
    if (!ledc_timer_inited) {
    ledc_timer_config_t ledc_timer;
    memset(&ledc_timer, 0, sizeof(ledc_timer));
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
            ledc_timer.duty_resolution = LEDC_TIMER_13_BIT; // 13-bit resolution
        ledc_timer.timer_num = ServoController::ledc_timer_num;
        ledc_timer.freq_hz = 50;
        ledc_timer.clk_cfg = LEDC_AUTO_CLK;
        esp_err_t err = ledc_timer_config(&ledc_timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "LEDC timer config failed (err=0x%x)", err);
            return;
        }
        ledc_timer_inited = true;
    }

        // Try to find a free LEDC channel. Use a conservative bound matching
        // typical SOC configuration (8 channels). If other code has already
        // used some channels, iterate and pick the first one that succeeds.
        const int MAX_LEDC_CHANNELS = 8; // matches CONFIG_SOC_LEDC_CHANNEL_NUM in sdkconfig
        int channel = -1;
        for (int c = 0; c < MAX_LEDC_CHANNELS; ++c) {
            ledc_channel_config_t ch;
            memset(&ch, 0, sizeof(ch));
            ch.gpio_num = pin;
            ch.speed_mode = LEDC_LOW_SPEED_MODE;
            ch.channel = (ledc_channel_t)c;
            ch.intr_type = LEDC_INTR_DISABLE;
            ch.timer_sel = ServoController::ledc_timer_num;
            ch.duty = 0;
            ch.hpoint = 0;
            esp_err_t err = ledc_channel_config(&ch);
            if (err == ESP_OK) {
                channel = c;
                // advance next_ledc_channel so we try to spread allocations,
                // but don't rely on it as the single source of truth.
                if (c >= next_ledc_channel) next_ledc_channel = c + 1;
                break;
            }
            // if channel_config failed, try the next one
        }

        if (channel < 0) {
            ESP_LOGE(TAG, "No free LEDC channels available (tried %d)", MAX_LEDC_CHANNELS);
            return;
        }

    // mark as using the found channel
    use_ledc = true;
    ledc_channel = channel;

    // set center position
    setAngleLEDC(90);
}

void ServoController::setAngleLEDC(int angle) {
    // map to pulse width in microseconds
    uint32_t pulse = angleToCompare(angle);
    // period = 20ms = 20000us
    const uint32_t period_us = 20000;
    const uint32_t scale = (1 << LEDC_TIMER_13_BIT) - 1; // 13-bit resolution
    uint32_t duty = (pulse * scale) / period_us;
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ledc_channel, duty);
    if (err == ESP_OK) {
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ledc_channel);
    } else {
        ESP_LOGE(TAG, "LEDC set duty failed for channel %d (err=0x%x)", ledc_channel, err);
    }
}

void ServoController::setAngle(int angle) {
    if (angle == current_angle) return;

    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    ensureOperatorInitialized();
    if (use_ledc) {
        setAngleLEDC(angle);
        current_angle = angle;
        return;
    }
    if (!comparator) {
        ESP_LOGE(TAG, "Comparator not available for GPIO %d; cannot set angle", pin);
        return;
    }

    uint32_t cmp = angleToCompare(angle);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, cmp));
    current_angle = angle;
}

int ServoController::getCurrentAngle() {
    return current_angle;
}

extern "C" void servos_stop(void) {
    ESP_LOGI(MOTOR_TAG, "Stopping all servos");
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
}

void ServoController::stop() {
    ESP_LOGI(TAG, "Stopping servo on GPIO %d", pin);
    if (use_ledc && ledc_channel >= 0) {
        // Stop LEDC output and release the channel
        esp_err_t err = ledc_stop(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ledc_channel, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ledc_stop failed for channel %d (err=0x%x)", ledc_channel, err);
        }
        use_ledc = false;
        ledc_channel = -1;
        return;
    }

    // If MCPWM resources are used, delete comparator/generator so output stops
    if (generator) {
        esp_err_t err = mcpwm_del_generator(generator);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "mcpwm_del_generator failed for GPIO %d (err=0x%x)", pin, err);
        }
        generator = NULL;
    }
    if (comparator) {
        esp_err_t err = mcpwm_del_comparator(comparator);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "mcpwm_del_comparator failed for GPIO %d (err=0x%x)", pin, err);
        }
        comparator = NULL;
    }
}