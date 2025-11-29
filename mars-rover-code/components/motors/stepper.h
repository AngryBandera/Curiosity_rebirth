#pragma once

#include "motors_cfg.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

class Stepper {
public:
	Stepper();
	~Stepper();

	// Initialize GPIO pins and basic parameters. This does not start the
	// background task used for continuous movement.
	esp_err_t init(int step_pin = A4988::STEP_PIN,
				   int dir_pin = A4988::DIR_PIN,
				   int enable_pin = A4988::ENABLE_PIN,
				   int steps_per_rev = 200,
				   int microstep = 16,
				   int default_pulse_us = A4988::STEP_PULSE_US);

	// Start/stop the stepper control task. The control task will move the
	// motor toward `target_angle_deg` in the background.
	esp_err_t start_task(const char* name = "stepper", int core = 0, int prio = 5, size_t stack = 4096);
	void stop_task();

	// Simple synchronous helpers (compatible API):
	void set_direction(bool dir);
	void step_once(int pulse_us = A4988::STEP_PULSE_US);
	void step_once_async(int pulse_us = A4988::STEP_PULSE_US);

	// High level control:
	// Set absolute target angle in degrees (0..360). Task will move there.
	void set_target_angle(float angle_deg, int steps_per_sec = 200);
	float get_current_angle();

private:
	// internal helpers
	void task_entry();
	static void task_entry_trampoline(void* arg);
	void do_step_pulse(int pulse_us);

	// pins and params
	int step_pin_;
	int dir_pin_;
	int enable_pin_;
	int steps_per_rev_;
	int microstep_;
	int pulse_us_default_;

	// positioning
	volatile int64_t position_steps_; // current position in microsteps
	volatile int64_t target_steps_;
	volatile int32_t target_rate_sps_; // steps per second while moving

	// RTOS
	TaskHandle_t task_handle_;
	SemaphoreHandle_t lock_;
	volatile bool task_running_;
};
