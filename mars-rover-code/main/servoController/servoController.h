#pragma once

#include "driver/mcpwm_prelude.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <stdint.h>

class ServoController {
public:
    static void init_shared_timer(int group_id = 0);
    ServoController(int gpio_pin, int min_pulse_us = 500, int max_pulse_us = 2500);

    ~ServoController();

    void setAngle(int angle);
    void stop();

    int getCurrentAngle();

private:
    static const char* TAG;
    static mcpwm_timer_handle_t timer;
    static mcpwm_oper_handle_t shared_oper;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;
    int pin;
    int current_angle;
    int min_pulse;
    int max_pulse;
    void ensureOperatorInitialized();
    uint32_t angleToCompare(int angle) const;
    // LEDC fallback when MCPWM resources are unavailable
    bool use_ledc;
    int ledc_channel;
    static bool ledc_timer_inited;
    static int next_ledc_channel;
    static ledc_timer_t ledc_timer_num;
    void initLEDC();
    void setAngleLEDC(int angle);
};