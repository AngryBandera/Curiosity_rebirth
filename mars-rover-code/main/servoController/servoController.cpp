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

#define SERVO_CHANNEL_0 0
#define SERVO_CHANNEL_1 1
#define SERVO_CHANNEL_2 2
#define SERVO_CHANNEL_3 3

#define SERVO_LEFT_ANGLE   0
#define SERVO_RIGHT_ANGLE  180
#define SERVO_CENTER_ANGLE 90

const servo_config_t servo_cfg = {
    /* max_angle */ (uint16_t)180,
    /* min_width_us */ (uint16_t)500,
    /* max_width_us */ (uint16_t)2500,
    /* freq */ (uint32_t)50,
    /* timer_number */ (ledc_timer_t)LEDC_TIMER_0,
    /* channels */ {
        /* servo_pin[] */ {
            (gpio_num_t)FRONT_LEFT_SERVO_PIN,
            (gpio_num_t)FRONT_RIGHT_SERVO_PIN,
            (gpio_num_t)BACK_LEFT_SERVO_PIN,
            (gpio_num_t)BACK_RIGHT_SERVO_PIN,
            /* remaining entries (if any) will be zero-initialized) */
        },
        /* ch[] */ {
            (ledc_channel_t)LEDC_CHANNEL_0,
            (ledc_channel_t)LEDC_CHANNEL_1,
            (ledc_channel_t)LEDC_CHANNEL_2,
            (ledc_channel_t)LEDC_CHANNEL_3,
        }
    },
    /* channel_number */ (uint8_t)4,
};

static const char* MOTOR_TAG = "MOTORS";

static ServoController frontLeft(FRONT_LEFT_SERVO_PIN, SERVO_CHANNEL_0);
static ServoController frontRight(FRONT_RIGHT_SERVO_PIN, SERVO_CHANNEL_1);
static ServoController backLeft(BACK_LEFT_SERVO_PIN, SERVO_CHANNEL_2);
static ServoController backRight(BACK_RIGHT_SERVO_PIN, SERVO_CHANNEL_3);

extern "C" void servos_init(void) {
    ESP_ERROR_CHECK(iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg));
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

ServoController::ServoController(int gpio_pin, int channel): pin(gpio_pin), channel(channel) {
    ESP_LOGI(TAG, "ServoController channel %d constructed for GPIO %d", channel, gpio_pin);
}   

void ServoController::setAngle(int angle) {
    if (angle == current_angle) return;

    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, channel, angle);
    current_angle = angle;
    return;
}

int ServoController::getCurrentAngle() {
    return current_angle;
}