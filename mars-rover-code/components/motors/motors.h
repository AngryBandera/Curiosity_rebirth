#include "driver/gpio.h"

class WheelDriver {
private:
    gpio_num_t pin1;
    gpio_num_t pin2;
    const char *TAG;

public:
    WheelDriver(gpio_num_t pin_1, gpio_num_t pin_2, const char *new_TAG);
};
