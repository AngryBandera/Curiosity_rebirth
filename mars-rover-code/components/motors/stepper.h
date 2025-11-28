#pragma once

#include "motors_cfg.h"
#include "esp_err.h"

class Stepper {
public:
	static esp_err_t init(int step_pin = A4988::STEP_PIN, int dir_pin = A4988::DIR_PIN, int enable_pin = A4988::ENABLE_PIN);

	static void set_direction(bool dir);

	static void step_once(int pulse_us = A4988::STEP_PULSE_US);

	static void step_once_async(int pulse_us = A4988::STEP_PULSE_US);
};
