#ifndef PCA_BUFFER
#define PCA_BUFFER

#include "i2cdev.h"
#include <stdint.h>

// I2C and PCA9685 definitions
#define I2C_MASTER_SCL_IO    GPIO_NUM_22    // GPIO for SCL
#define I2C_MASTER_SDA_IO    GPIO_NUM_21    // GPIO for SDA
#define I2C_MASTER_FREQ_HZ   100000
#define PCA9685_ADDR         PCA9685_ADDR_BASE  // 0x40

// Servo constants будуть включені через motors_cfg.h
// Не дублюємо тут!

class PCA9685Buffer {
private:
    i2c_dev_t* device;
    uint16_t buffer[16];  // Буфер для 16 каналів (0-4095)
    bool dirty;           // Чи є незбережені зміни
    const char* TAG = "PCA9685Buffer";
    
public:
    PCA9685Buffer(i2c_dev_t* pca9685, gpio_num_t I2C_SDA, gpio_num_t I2C_SCL);
    ~PCA9685Buffer();
    
    void set_channel_value(uint8_t channel, uint16_t value);
    uint16_t get_channel_value(uint8_t channel);
    void flush();
    void set_channel_immediate(uint8_t channel, uint16_t value);
    
    /*
     * Чи є незбережені зміни
     */
    bool is_dirty() const;
    
    /*
     * Очистити буфер (встановити всі канали в 0)
     */
    void clear();
};

#endif
