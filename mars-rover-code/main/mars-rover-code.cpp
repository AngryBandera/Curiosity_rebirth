#include "esp_err.h"
#include "esp_log.h"
#include "freertos/projdefs.h"
#include "hal/ledc_types.h"
#include "motors.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "portmacro.h"
#include "soc/clk_tree_defs.h"
#include "pca9685.h"
#include "soc/gpio_num.h"
#include <cstdint>
#include <sys/types.h>

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
    ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&pca9685, 50));


    /*constexpr uint16_t MIN_DUTY = (500 * 4096) / 20000;   // ~102 (500 μs)
    constexpr uint16_t MAX_DUTY = (2500 * 4096) / 20000;  // ~512 (2500 μs)

    while (1) {
        for (uint16_t pos = 70; pos <= MAX_DUTY; pos += 1) {
            pca9685_set_pwm_value(&pca9685, 0, pos);
            vTaskDelay(pdMS_TO_TICKS(50));
            ESP_LOGI("SERVO_POS", "duty=%d, pulse≈%d μs", 
                    pos, (pos * 20000) / 4096);
        }


        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }*/

    
    DriveSystem rover{&pca9685};

    /*float angle = 0.0f;
    while (true) {
        rover.print_angles();
        rover.rotate(angle);
        angle += 1.0f;
        ESP_LOGI("be", "Angle: %.2f", angle);

        if (angle > 45.0f) angle = -45.0f;
        vTaskDelay(pdMS_TO_TICKS(50));

    }*/
    rover.rotate(0.0f);

    vTaskDelay(pdMS_TO_TICKS(1000));
    

    for (float i = 0.0f; i < 45.0f; i += 3.0f) {
        rover.rotate(i);
        vTaskDelay(pdMS_TO_TICKS(100));
    };
    rover.rotate(45.0f);
    vTaskDelay(pdMS_TO_TICKS(1000));
    

    for (float i = 45.0f; i > 0.0f; i -= 4.0f) {
        rover.rotate(i);
        vTaskDelay(pdMS_TO_TICKS(100));
    };


    ESP_LOGI("MAIN", "LOOOOP");

    

}
