#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"

#include "camera_server.h"
#include "command_get.h"
#include "driver/gpio.h"

#define GPIO1 GPIO_NUM_13
#define GPIO2 GPIO_NUM_14

static const char* TAG = "CAMERA_TEST";
const pins_t pins = {GPIO1, GPIO2};


// -------------------------
// WiFi AP MODE
// -------------------------
static void init_wifi_ap()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_config = {};
    strcpy((char*)ap_config.ap.ssid, "ESP32_CAM_AP");
    strcpy((char*)ap_config.ap.password,"12345678");

    ap_config.ap.ssid_len = strlen("ESP32_CAM_AP");
    ap_config.ap.channel = 1;
    ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    ap_config.ap.max_connection = 4;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP started. Connect to: http://192.168.4.1/");
}


extern "C" void app_main(void)
{
    // ---------- INIT NVS ----------
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // ---------- WIFI ----------
    init_wifi_ap();

    // ---------- CAMERA ----------
    if (!initCamera(NULL)) {
        ESP_LOGE(TAG, "Camera initialization FAILED");
        return;
    }
    ESP_LOGI(TAG, "Camera OK");

    // ---------- WEB SERVER ----------
    if (!initWebServer(80)) {
        ESP_LOGE(TAG, "Web server failed");
        return;
    }

    ESP_LOGI(TAG, "WEB OK");


    // ---------- GPIO INPUT INIT ----------
    gpio_set_direction(GPIO1, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO2, GPIO_MODE_INPUT);

    ESP_LOGI(TAG, "READY. Waiting for GPIO commands...");


    // ---------- MAIN LOOP ----------
    startVideoStream();
    while (true) {
        uint8_t command = get_pins_status(&pins);

        do_task_based_on_pins(command);

        ESP_LOGI(TAG, "CAMERA STATUS: %s", getCameraStatus());

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
