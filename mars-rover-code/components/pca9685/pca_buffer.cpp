#include "pca_buffer.h"
#include "pca9685.h"
#include "../motors/motors_cfg.h"  // <-- додано
#include <esp_log.h>
#include <cstring>

PCA9685Buffer::PCA9685Buffer(i2c_dev_t* pca9685):
    device{pca9685},
    dirty{false}
{
    ESP_ERROR_CHECK(i2cdev_init());

    memset(device, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(pca9685_init_desc(device, PCA9685_ADDR, 
                                    I2C_NUM_0, 
                                    I2C_MASTER_SDA_IO, 
                                    I2C_MASTER_SCL_IO));

    ESP_ERROR_CHECK(pca9685_init(device));
    ESP_ERROR_CHECK(pca9685_set_pwm_frequency(device, Servo::FREQ));

    clear();
    flush();
}

void PCA9685Buffer::set_channel_value(uint8_t channel, uint16_t value) {
    if (channel >= 16) {
        ESP_LOGE(TAG, "Invalid channel %d", channel);
        return;
    }
    if (value > 4095) {
        ESP_LOGW(TAG, "clamping PWM %u->4095 for channel %u", value, channel);
        value = 4095;
    }
    buffer[channel] = value;
    dirty = true;
}

uint16_t PCA9685Buffer::get_channel_value(uint8_t channel) {
    if (channel >= 16) {
        ESP_LOGE(TAG, "Invalid channel %d", channel);
        return 0;
    }
    return buffer[channel];
}

void PCA9685Buffer::flush() {
    if (!dirty) return;
    ESP_ERROR_CHECK(pca9685_set_pwm_values(device, 0, 16, buffer));
    dirty = false;
}

void PCA9685Buffer::set_channel_immediate(uint8_t channel, uint16_t value) {
    if (channel >= 16) {
        ESP_LOGE(TAG, "Invalid channel %d", channel);
        return;
    }
    if (value > 4095) {
        ESP_LOGW(TAG, "clamping PWM %u->4095 for channel %u", value, channel);
        value = 4095;
    }
    ESP_ERROR_CHECK(pca9685_set_pwm_value(device, channel, value));
}

bool PCA9685Buffer::is_dirty() const {
    return dirty;
}

void PCA9685Buffer::clear() {
    for (uint8_t i = 0; i < 4; i++)  buffer[i] = Servo::CENTER_DUTY;
    for (uint8_t i = 4; i < 16; i++) buffer[i] = 0;
    dirty = true;
}
