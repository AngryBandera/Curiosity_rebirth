#include "comm_espnow.h"
#include "esp_log.h"
#include "esp_mac.h"
#include <string.h>
#include <stdio.h>

static const char* TAG = "ESP_NOW_COMM";

// === Глобальні змінні ===
static bool espnow_initialized = false;
static uint8_t controller_mac[6] = {0};
static command_callback_t user_callback = nullptr;
static esp_now_send_cb_t send_callback = nullptr;

// === Приватні допоміжні ===
static void onDataReceivedInternal(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len);
static void onDataSentInternal(const uint8_t* mac_addr, esp_now_send_status_t status);

// === Ініціалізація ===
bool initESPNow(const uint8_t* controller_mac_addr) {
    if (espnow_initialized) {
        ESP_LOGI(TAG, "ESP-NOW already initialized");
        return true;
    }

    if (controller_mac_addr == nullptr) {
        ESP_LOGE(TAG, "Controller MAC is NULL");
        return false;
    }

    memcpy(controller_mac, controller_mac_addr, 6);
    ESP_LOGI(TAG, "Controller MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             controller_mac[0], controller_mac[1], controller_mac[2],
             controller_mac[3], controller_mac[4], controller_mac[5]);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_err_t ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(ret));
        return false;
    }

    // Реєструємо колбеки
    ret = esp_now_register_recv_cb(onDataReceivedInternal);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register receive callback: %s", esp_err_to_name(ret));
        esp_now_deinit();
        return false;
    }

    ret = esp_now_register_send_cb(onDataSentInternal);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register send callback: %s", esp_err_to_name(ret));
        esp_now_deinit();
        return false;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, controller_mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer");
        esp_now_deinit();
        return false;
    }

    espnow_initialized = true;
    ESP_LOGI(TAG, "ESP-NOW initialized successfully");
    return true;
}

void deinitESPNow() {
    if (espnow_initialized) {
        esp_now_deinit();
        espnow_initialized = false;
        memset(controller_mac, 0, 6);
        ESP_LOGI(TAG, "ESP-NOW deinitialized");
    }
}

bool isESPNowInitialized() {
    return espnow_initialized;
}

// === Реєстрація callback ===
void registerReceiveCallback(command_callback_t callback) {
    user_callback = callback;
}

void registerSendCallback(esp_now_send_cb_t callback) {
    send_callback = callback;
}

// === Обробка прийому ===
static void onDataReceivedInternal(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len) {
    if (!data || len < sizeof(command_packet_t)) {
        ESP_LOGW(TAG, "Received invalid data (len=%d)", len);
        return;
    }

    command_packet_t cmd = parseCommand(data, len);
    if (!isValidCommand(&cmd)) {
        ESP_LOGW(TAG, "Invalid command received");
        return;
    }

    ESP_LOGI(TAG, "Received command: type=%d, param1=%d, param2=%d", cmd.type, cmd.param1, cmd.param2);

    if (user_callback) {
        user_callback(&cmd);
    }
}

// === Обробка відправки ===
static void onDataSentInternal(const uint8_t* mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Send status to %02X:%02X:%02X:%02X:%02X:%02X -> %s",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5],
             (status == ESP_NOW_SEND_SUCCESS) ? "SUCCESS" : "FAIL");

    if (send_callback) {
        send_callback(mac_addr, status);
    }
}

// === Парсинг команд ===
command_packet_t parseCommand(const uint8_t* data, int len) {
    command_packet_t cmd = {};
    if (len >= sizeof(command_packet_t)) {
        memcpy(&cmd, data, sizeof(command_packet_t));
    }
    return cmd;
}

bool isValidCommand(const command_packet_t* cmd) {
    if (!cmd) return false;
    return (cmd->type >= CMD_CAPTURE_PHOTO && cmd->type <= CMD_CHANGE_QUALITY);
}

// === Відправка статусу ===
bool sendStatusUpdate(status_type_t status) {
    status_packet_t pkt = {};
    pkt.status = status;
    pkt.timestamp = esp_log_timestamp();

    esp_err_t result = esp_now_send(controller_mac, (uint8_t*)&pkt, sizeof(pkt));
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send status: %s", esp_err_to_name(result));
        return false;
    }

    ESP_LOGI(TAG, "Status sent: %d", status);
    return true;
}

bool sendStatusWithData(const status_packet_t* status_packet) {
    if (!status_packet) return false;

    esp_err_t result = esp_now_send(controller_mac, (uint8_t*)status_packet, sizeof(status_packet_t));
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send status with data: %s", esp_err_to_name(result));
        return false;
    }

    ESP_LOGI(TAG, "Status with data sent");
    return true;
}

// === Робота з peers ===
bool addPeer(const uint8_t* peer_mac) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peer_mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    esp_err_t res = esp_now_add_peer(&peerInfo);
    if (res == ESP_OK) {
        ESP_LOGI(TAG, "Peer added");
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(res));
        return false;
    }
}

bool removePeer(const uint8_t* peer_mac) {
    esp_err_t res = esp_now_del_peer(peer_mac);
    if (res == ESP_OK) {
        ESP_LOGI(TAG, "Peer removed");
        return true;
    } else {
        ESP_LOGE(TAG, "Failed to remove peer: %s", esp_err_to_name(res));
        return false;
    }
}

bool isPeerAdded(const uint8_t* peer_mac) {
    return esp_now_is_peer_exist(peer_mac);
}

// === MAC адреси ===
void getLocalMacAddress(uint8_t* mac) {
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
}

void getControllerMacAddress(uint8_t* mac) {
    memcpy(mac, controller_mac, 6);
}
