#pragma once

#include "driver/mcpwm_prelude.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "iot_servo.h"
#include <stdint.h>

class ServoController {
public:
    ServoController(int gpio_pin, int channel);

    void setAngle(int angle);

    int getCurrentAngle();

private:
    static const char* TAG;
    int pin;
    int channel;
    int current_angle = -1;
};