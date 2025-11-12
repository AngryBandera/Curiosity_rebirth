//
// Created by bogda on 12/11/2025.
//

#ifndef CURIOSITY_REBIRTH_COMM_ESPNOW_H
#define CURIOSITY_REBIRTH_COMM_ESPNOW_H

#include "esp_now.h"
#include "esp_wifi.h"
#include <stdint.h>
#include <stdbool.h>

// Типи команд що мож
уть приходити від контролера
typedef enum {
    CMD_CAPTURE_PHOTO = 0x01,
    CMD_START_STREAM = 0x02,
    CMD_STOP_STREAM = 0x03,
    CMD_GET_STATUS = 0x04,
    CMD_CHANGE_QUALITY = 0x05,
    CMD_UNKNOWN = 0xFF
} command_type_t;

// Типи статусів для відправки назад
typedef enum {
    STATUS_OK = 0x00,
    STATUS_ERROR = 0x01,
    STATUS_BUSY = 0x02,
    STATUS_PHOTO_CAPTURED = 0x03,
    STATUS_STREAM_STARTED = 0x04,
    STATUS_STREAM_STOPPED = 0x05
} status_type_t;

// Структура команди
typedef struct {
    command_type_t type;
    uint8_t param1;  // додаткові параметри
    uint8_t param2;
    uint8_t param3;
    uint32_t timestamp;
} command_packet_t;

// Структура статусу для відправки
typedef struct {
    status_type_t status;
    uint8_t data[16];  // додаткові дані
    uint32_t timestamp;
} status_packet_t;

// Callback тип для обробки команд
typedef void (*command_callback_t)(const command_packet_t* cmd);

// Функції для ініціалізації ESP-NOW
bool initESPNow(const uint8_t* controller_mac);
void deinitESPNow();
bool isESPNowInitialized();

// Реєстрація callback функцій
void registerReceiveCallback(command_callback_t callback);
void registerSendCallback(esp_now_send_cb_t callback);

// Callback функція що викликається при отриманні даних
void onDataReceived(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len);

// Функція для відправки callback (опціонально)
void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);

// Парсинг команд
command_packet_t parseCommand(const uint8_t* data, int len);
bool isValidCommand(const command_packet_t* cmd);

// Відправка статусу назад контролеру
bool sendStatusUpdate(status_type_t status);
bool sendStatusWithData(const status_packet_t* status_packet);

// Додавання пристроїв (peers)
bool addPeer(const uint8_t* peer_mac);
bool removePeer(const uint8_t* peer_mac);
bool isPeerAdded(const uint8_t* peer_mac);

// Отримання MAC адреси
void getLocalMacAddress(uint8_t* mac);
void getControllerMacAddress(uint8_t* mac);

#endif //CURIOSITY_REBIRTH_COMM_ESPNOW_H