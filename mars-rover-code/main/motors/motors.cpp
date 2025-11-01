#include "motors.h"
#include "motorsInterface.h"
#include "driver/gpio.h"
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

WheelDriver::WheelDriver(gpio_num_t pin_1, gpio_num_t pin_2, const char *new_TAG)
    : pin1{pin_1},
      pin2{pin_2},
      TAG{new_TAG}
{
  gpio_config_t pins_conf{
      .pin_bit_mask = (1ULL << pin1) | (1ULL << pin2),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&pins_conf);

  gpio_set_level(pin1, 0);
  gpio_set_level(pin2, 0);

  ESP_LOGE(TAG, "Configured pins (%d and %d) for motor", pin1, pin2);
}
