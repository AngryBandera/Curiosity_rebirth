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

static const char* TAG = "MARS_ROVER_MAIN";

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


    // За замовчуванням запускаємо стрім, щоб одразу бачити картинку
    // startVideoStream();

    // 6. Main Loop
    while (true) {
        // Лог статусу кожні 5 секунд (щоб не засмічувати консоль)
        static uint32_t loop_cnt = 0;
        if (loop_cnt++ % 10 == 0) { // 10 * 500ms = 5 sec
            ESP_LOGI(TAG, "[Status] Cam: %s | Stream: %s | Last Cmd: %d", 
                     isCameraInitialized() ? "OK" : "ERR",
                     isStreaming() ? "ON" : "OFF");
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Перевірка команд 2 рази на секунду
    }
}