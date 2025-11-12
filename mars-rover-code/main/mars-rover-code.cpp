#include "esp_err.h"
#include "esp_log.h"
#include "hal/ledc_types.h"
#include "motors.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "portmacro.h"
#include "soc/clk_tree_defs.h"
#include "pca9685.h"
#include <cstdint>

#define SERVO_FREQ 50
#define SERVO_RESOLUTION LEDC_TIMER_14_BIT

#define I2C_MASTER_SCL_IO    GPIO_NUM_22    // GPIO для SCL
#define I2C_MASTER_SDA_IO    GPIO_NUM_21    // GPIO для SDA
#define I2C_MASTER_FREQ_HZ   100000
#define PCA9685_ADDR         PCA9685_ADDR_BASE  // 0x40


extern "C" void app_main(void)
{
    i2c_dev_t pca9685;
    
    ESP_ERROR_CHECK(i2cdev_init());
    
    memset(&pca9685, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(pca9685_init_desc(&pca9685, PCA9685_ADDR, 
                                      I2C_NUM_0, 
                                      I2C_MASTER_SDA_IO, 
                                      I2C_MASTER_SCL_IO));

    ESP_ERROR_CHECK(pca9685_init(&pca9685));
    ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&pca9685, 500));

    
    while (1) {
        /*for (uint16_t pos = 1000; pos<4012; pos+=10) {
            pca9685_set_pwm_value(&pca9685, 0, pos);
            vTaskDelay(pdMS_TO_TICKS(10));
            ESP_LOGI("BEBE", "%d", pos);
        }*/
        pca9685_set_pwm_value(&pca9685, 0, 4096);
        

        ESP_LOGI("aaa", "cycle ended");

    }


    ledc_timer_config_t servo_timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false
    };

    ESP_ERROR_CHECK(ledc_timer_config(&servo_timer_conf));


    ledc_timer_config_t dc_timer_config = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&dc_timer_config));
    // gpio_set_direction();

    
    DriveSystem rover{LEDC_TIMER_1, LEDC_TIMER_0};

    while (true) {

        for (float i = -40.0f; i <= 40.0f; i++) {
            rover.rotate(i);
            ESP_LOGI("Angle", "%.2f", i);
            rover.print_angles();
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }

        for (float i = 40.0f; i >= -40.0f; i--) {
            rover.rotate(i);
            ESP_LOGI("Angle", "%.2f", i);
            rover.print_angles();
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }

    }

    /*WheelMotor motor{GPIO_NUM_5, GPIO_NUM_6, "TAG", LEDC_CHANNEL_0};

    WheelMotor::shared_timer = LEDC_TIMER_0;
    while (true) {
        for (uint8_t i = 0; i < 255; i++) {
            motor.forward(255);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            ESP_LOGI("motor", "%d", i);
        }

        for (uint8_t i = 255; i > 0; i--) {
            motor.backward(255);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }*/
}
