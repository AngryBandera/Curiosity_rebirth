// main/camera_test.cpp
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"

#include "camera_server.h"

static const char* TAG = "CAMERA_TEST";

static void init_wifi_ap()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_config = {};
    strncpy((char*)ap_config.ap.ssid, "ESP32_CAM_AP", sizeof(ap_config.ap.ssid) - 1);
    strncpy((char*)ap_config.ap.password, "12345678", sizeof(ap_config.ap.password) - 1);
    ap_config.ap.ssid_len = strlen("ESP32_CAM_AP");
    ap_config.ap.channel = 1;
    ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    ap_config.ap.ssid_hidden = 0;
    ap_config.ap.max_connection = 4;
    ap_config.ap.beacon_interval = 100;

    if (strlen((char*)ap_config.ap.password) == 0)
        ap_config.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "📡 WiFi AP started");
    ESP_LOGI(TAG, "   SSID: %s", ap_config.ap.ssid);
    ESP_LOGI(TAG, "   Password: %s", ap_config.ap.password);
    ESP_LOGI(TAG, "   IP: 192.168.4.1");
}

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_LOGI(TAG, "=== Mars Rover Camera System ===");

    init_wifi_ap();

    if (!initCamera(NULL)) {
        ESP_LOGE(TAG, "❌ Camera initialization failed!");
        return;
    }

    ESP_LOGI(TAG, "✅ Camera initialized successfully");

    if (!initWebServer(80)) {
        ESP_LOGE(TAG, "❌ Failed to start web server");
        return;
    }

    ESP_LOGI(TAG, "✅ Web server started");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "🌐 Connect to: http://192.168.4.1/");
    ESP_LOGI(TAG, "");

    // ВАЖЛИВО: Вмикаємо streaming ОДРАЗУ
    startVideoStream();

    uint32_t loop_count = 0;
    
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        loop_count++;
        
        const char* status = getCameraStatus();
        ESP_LOGI(TAG, "[%lu] Status: %s | Streaming: %s", 
                 loop_count, 
                 status,
                 isStreaming() ? "ON" : "OFF");
    }
}