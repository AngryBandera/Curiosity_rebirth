#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Ваші заголовні файли
#include "camera_server.h"

// --- НАЛАШТУВАННЯ ПІНІВ ДЛЯ КОМАНД ---
#define GPIO_CMD_BIT_0 GPIO_NUM_13
#define GPIO_CMD_BIT_1 GPIO_NUM_14

static const char* TAG = "MARS_ROVER_MAIN";
const pins_t pins = {GPIO_CMD_BIT_0, GPIO_CMD_BIT_1}; // Структура з твого command_get.h

// -------------------------
// WIFI CONFIGURATION
// -------------------------
static void init_wifi_ap()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_config = {};
    strncpy((char*)ap_config.ap.ssid, "MARS_ROVER_CAM", sizeof(ap_config.ap.ssid) - 1);
    strncpy((char*)ap_config.ap.password, "mars2025", sizeof(ap_config.ap.password) - 1);
    
    ap_config.ap.ssid_len = strlen((char*)ap_config.ap.ssid);
    ap_config.ap.channel = 1;
    ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    ap_config.ap.max_connection = 4;
    ap_config.ap.beacon_interval = 100;

    if (strlen((char*)ap_config.ap.password) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "📡 WiFi AP Started!");
    ESP_LOGI(TAG, "   SSID: %s", ap_config.ap.ssid);
    ESP_LOGI(TAG, "   PASS: %s", ap_config.ap.password);
    ESP_LOGI(TAG, "   URL:  http://192.168.4.1");
    ESP_LOGI(TAG, "========================================");
}

// -------------------------
// COMMAND LOGIC
// -------------------------
// Функція для обробки команд (симуляція do_task_based_on_pins, якщо її немає в хедері)
// 0 = 00 = IDLE
// 1 = 01 = START STREAM
// 2 = 10 = STOP STREAM
// 3 = 11 = TAKE PHOTO
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
void process_rover_command(uint8_t command) {
    static uint8_t last_command = 255; // Щоб не спамити логами
    
    // Якщо команда "Фото" (3), ми виконуємо її завжди, навіть якщо вона повторюється.
    // Для інших команд виконуємо тільки при зміні стану.
    if (command == last_command && command != 3) {
        return; 
    }
    
    last_command = command;

    switch (command) {
        case 0: // 00
            // Idle state - нічого не робимо, просто чекаємо
            // ESP_LOGD(TAG, "Command: IDLE");
            break;

        case 1: // 01
            if (!isStreaming()) {
                ESP_LOGI(TAG, "🚀 Command received: START STREAM");
                startVideoStream();
            }
            break;

        case 2: // 10
            if (isStreaming()) {
                ESP_LOGI(TAG, "🛑 Command received: STOP STREAM");
                stopVideoStream();
            }
            break;

        case 3: // 11
            ESP_LOGI(TAG, "📸 Command received: TAKE PHOTO");
            // Викликаємо нашу нову функцію
            if (take_photo_internal()) {
                ESP_LOGI(TAG, ">> Photo captured successfully via PIN command");
            } else {
                ESP_LOGE(TAG, ">> Photo capture failed (Camera busy?)");
            }
            // Робимо невелику паузу, щоб не зробити 100 фото за секунду, поки пін затиснутий
            vTaskDelay(pdMS_TO_TICKS(1000)); 
            break;

        default:
            ESP_LOGW(TAG, "Unknown command: %d", command);
            break;
    }
}

// -------------------------
// MAIN
// -------------------------
extern "C" void app_main(void)
{
    // 1. NVS Init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_LOGI(TAG, "Initializing Mars Rover System...");

    // 2. WiFi
    init_wifi_ap();

    // 3. Camera Setup
    if (!initCamera(NULL)) {
        ESP_LOGE(TAG, "❌ Camera init FAILED! Check connection.");
        // Блимаємо світлодіодом помилки або перезавантажуємось
        return;
    }
    ESP_LOGI(TAG, "✅ Camera Sensor OK");

    // 4. Web Server
    if (!initWebServer(80)) {
        ESP_LOGE(TAG, "❌ Web Server FAILED!");
        return;
    }
    ESP_LOGI(TAG, "✅ Web Server Running on Port 80 & 81");

    // 5. GPIO Config
    // ВАЖЛИВО: Налаштуємо піни тут, щоб бути впевненими
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_CMD_BIT_0) | (1ULL << GPIO_CMD_BIT_1);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE; // Або PULLUP, залежить від вашої схеми!
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "✅ GPIO Configured. Waiting for commands...");
    ESP_LOGI(TAG, "   Bit 0: GPIO %d", GPIO_CMD_BIT_0);
    ESP_LOGI(TAG, "   Bit 1: GPIO %d", GPIO_CMD_BIT_1);

    // За замовчуванням запускаємо стрім, щоб одразу бачити картинку
    // startVideoStream();

    // 6. Main Loop
    while (true) {
        // Отримуємо статус пінів
        // Припускаємо, що get_pins_status повертає десяткове значення (0, 1, 2, 3)
        // на основі двійкового коду з пінів.
        uint8_t command = get_pins_status(&pins);

        // Виконуємо логіку
        process_rover_command(command);

        // Лог статусу кожні 5 секунд (щоб не засмічувати консоль)
        static uint32_t loop_cnt = 0;
        if (loop_cnt++ % 10 == 0) { // 10 * 500ms = 5 sec
            ESP_LOGI(TAG, "[Status] Cam: %s | Stream: %s | Last Cmd: %d", 
                     isCameraInitialized() ? "OK" : "ERR",
                     isStreaming() ? "ON" : "OFF",
                     command);
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Перевірка команд 2 рази на секунду
    }
}