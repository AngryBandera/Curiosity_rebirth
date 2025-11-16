#include <stdint.h>
#include "command_get.h"
#include "driver/gpio.h"
#include "camera_server.h"
#include "esp_http_client.h"


void sendPhotoToServer(photo_data_t photo)
{
    esp_http_client_config_t config = {
        .url = "http://192.168.4.2/upload",
        .method = HTTP_METHOD_POST,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_header(client, "Content-Type", "image/jpeg");
    esp_http_client_set_post_field(client, (const char*)photo.buffer, photo.length);

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI("PHOTO", "Photo sent successfully");
    } else {
        ESP_LOGE("PHOTO", "Failed to send photo: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}


uint8_t get_pins_status(const pins_t *pins){
    gpio_num_t pins_array[2] = {
        pins->pin1,
        pins->pin2
    };

    uint8_t mask = 0;
    for (int i = 0; i < 2; i++){
        int state = gpio_get_level(pins_array[i]);
        mask |= (state << i);
    };
    return mask;

}

int do_task_based_on_pins(uint8_t task_number){
    if (task_number == 0x01){
        startVideoStream();
    }
    else if (task_number == 0x02){
        photo_data_t photo = capturePhoto();
        if (photo.buffer != NULL) {
            sendPhotoToServer(photo);
            releasePhotoBuffer(photo);
        }
    }
    else if (task_number == 0x03){
        stopVideoStream();
    };
    return 0;
};
