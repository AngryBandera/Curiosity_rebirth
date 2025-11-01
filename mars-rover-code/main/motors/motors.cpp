#include "motors.h"
#include "esp_log.h"
#include "servoController/servoInterface.h"

static const char *MOTOR_TAG = "MOTOR_CONTROL";


void moveForward() {
    ESP_LOGI(MOTOR_TAG, "Moving Forward");
}

void moveBackward() {
    ESP_LOGI(MOTOR_TAG, "Moving Backward");
}

void turnLeftMotors() {
    ESP_LOGI(MOTOR_TAG, "Turning Left");
}

void turnRightMotors() {
    ESP_LOGI(MOTOR_TAG, "Turning Right");
}

void stopMotors() {
    ESP_LOGI(MOTOR_TAG, "Stopping Motors");
}
