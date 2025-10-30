#include "motors.h"
#include "esp_log.h"

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

void frontServoLeft() {
    ESP_LOGI(MOTOR_TAG, "Front Servo Left");
}

void frontServoRight() {
    ESP_LOGI(MOTOR_TAG, "Front Servo Right");
}

void backServoLeft() {
    ESP_LOGI(MOTOR_TAG, "Back Servo Left");
}

void backServoRight() {
    ESP_LOGI(MOTOR_TAG, "Back Servo Right");
}