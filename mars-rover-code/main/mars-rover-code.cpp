#include "esp_log.h"

extern "C" [[noreturn]] void app_main(void)
{
    ESP_LOGE("LOG", "This is an error");
    // gpio_set_direction();
}
