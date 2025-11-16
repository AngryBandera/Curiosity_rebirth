// тут буде функціонал прийому даних від іншої esp
// Уточнити піни на яких можна робити рівні вверх та вниз
// треба для реалізації 3 команд два порта і кодувати 01, 10, 11 як команди

#ifndef CURIOSITY_REBIRTH_COMMAND_GET_H
#define CURIOSITY_REBIRTH_COMMAND_GET_H

#include <stdint.h>
#include "camera_server.h"
#include "driver/gpio.h"
#include "esp_http_client.h"
#include "esp_log.h"

typedef struct {
    gpio_num_t pin1;
    gpio_num_t pin2;
} pins_t;


void sendPhotoToServer(photo_data_t photo);

uint8_t get_pins_status(const pins_t *pins);

int do_task_based_on_pins(int task_number);

#endif
