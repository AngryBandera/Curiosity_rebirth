#include "camera_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char* TAG = "CAMERA_SERVER";

// Глобальні змінні
static httpd_handle_t server = NULL;
static bool camera_initialized = false;
static bool streaming_active = false;

// Нова система для capture зі стріму
static volatile bool capture_request_flag = false;
static photo_data_t captured_photo = {NULL, 0};
static volatile bool capture_ready = false;
static SemaphoreHandle_t capture_mutex = NULL;

// Конфігурація пінів для ESP32-WROVER з OV2640
static camera_config_t camera_config = {
    .pin_pwdn = -1,
    .pin_reset = -1,
    .pin_xclk = 21,
    .pin_sscb_sda = 26,
    .pin_sscb_scl = 27,
    
    .pin_d7 = 35,
    .pin_d6 = 34,
    .pin_d5 = 39,
    .pin_d4 = 36,
    .pin_d3 = 19,
    .pin_d2 = 18,
    .pin_d1 = 5,
    .pin_d0 = 4,
    .pin_vsync = 25,
    .pin_href = 23,
    .pin_pclk = 22,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_SVGA,
    .jpeg_quality = 12,
    .fb_count = 2,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY
};

// ============================================
// Функції для камери
// ============================================

bool initCamera(const camera_config_params_t* config) {
    if (camera_initialized) {
        ESP_LOGW(TAG, "Camera already initialized");
        return true;
    }

    // Створюємо мютекс для capture
    capture_mutex = xSemaphoreCreateMutex();
    if (!capture_mutex) {
        ESP_LOGE(TAG, "Failed to create capture mutex");
        return false;
    }

    // Якщо передали кастомну конфігурацію - використовуємо її
    if (config != NULL) {
        camera_config.pin_pwdn = config->pin_pwdn;
        camera_config.pin_reset = config->pin_reset;
        camera_config.pin_xclk = config->pin_xclk;
        camera_config.pin_sscb_sda = config->pin_sscb_sda;
        camera_config.pin_sscb_scl = config->pin_sscb_scl;
        camera_config.pin_d7 = config->pin_d7;
        camera_config.pin_d6 = config->pin_d6;
        camera_config.pin_d5 = config->pin_d5;
        camera_config.pin_d4 = config->pin_d4;
        camera_config.pin_d3 = config->pin_d3;
        camera_config.pin_d2 = config->pin_d2;
        camera_config.pin_d1 = config->pin_d1;
        camera_config.pin_d0 = config->pin_d0;
        camera_config.pin_vsync = config->pin_vsync;
        camera_config.pin_href = config->pin_href;
        camera_config.pin_pclk = config->pin_pclk;
        camera_config.frame_size = config->frame_size;
        camera_config.jpeg_quality = config->jpeg_quality;
        camera_config.fb_count = config->fb_count;
    }

    // Ініціалізація камери
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return false;
    }

    // Отримання sensor для додаткових налаштувань
    sensor_t* s = esp_camera_sensor_get();
    if (s != NULL) {
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
        s->set_special_effect(s, 0);
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_wb_mode(s, 0);
        s->set_exposure_ctrl(s, 1);
        s->set_aec2(s, 0);
        s->set_ae_level(s, 0);
        s->set_aec_value(s, 300);
        s->set_gain_ctrl(s, 1);
        s->set_agc_gain(s, 0);
        s->set_gainceiling(s, (gainceiling_t)0);
        s->set_bpc(s, 0);
        s->set_wpc(s, 1);
        s->set_raw_gma(s, 1);
        s->set_lenc(s, 1);
        s->set_hmirror(s, 0);
        s->set_vflip(s, 0);
        s->set_dcw(s, 1);
        s->set_colorbar(s, 0);
    }

    camera_initialized = true;
    ESP_LOGI(TAG, "Camera initialized successfully");
    return true;
}

photo_data_t capturePhoto() {
    photo_data_t photo = {NULL, 0};

    if (!camera_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        return photo;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        return photo;
    }

    photo.buffer = (uint8_t*)malloc(fb->len);
    if (photo.buffer != NULL) {
        memcpy(photo.buffer, fb->buf, fb->len);
        photo.length = fb->len;
        ESP_LOGI(TAG, "Photo captured: %d bytes", photo.length);
    } else {
        ESP_LOGE(TAG, "Failed to allocate memory for photo");
    }

    esp_camera_fb_return(fb);
    return photo;
}

void releasePhotoBuffer(photo_data_t photo) {
    if (photo.buffer != NULL) {
        free(photo.buffer);
    }
}

bool startVideoStream() {
    if (!camera_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        return false;
    }

    streaming_active = true;
    ESP_LOGI(TAG, "Video streaming started");
    return true;
}

bool stopVideoStream() {
    streaming_active = false;
    ESP_LOGI(TAG, "Video streaming stopped");
    return true;
}

bool isStreaming() {
    return streaming_active;
}

bool isCameraInitialized() {
    return camera_initialized;
}

const char* getCameraStatus() {
    if (!camera_initialized) {
        return "Camera not initialized";
    }
    if (streaming_active) {
        return "Streaming active";
    }
    return "Camera ready";
}

// ============================================
// Нові функції для capture зі стріму
// ============================================

void requestCaptureFromStream() {
    capture_request_flag = true;
    capture_ready = false;
    ESP_LOGI(TAG, "Capture requested from stream");
}

bool isCaptureReady() {
    return capture_ready;
}

photo_data_t getCapturedPhoto() {
    photo_data_t photo = {NULL, 0};
    
    if (xSemaphoreTake(capture_mutex, pdMS_TO_TICKS(100))) {
        if (capture_ready && captured_photo.buffer != NULL) {
            photo = captured_photo;
            // Очищаємо після отримання
            captured_photo.buffer = NULL;
            captured_photo.length = 0;
            capture_ready = false;
        }
        xSemaphoreGive(capture_mutex);
    }
    
    return photo;
}

// ============================================
// Функції для вебсервера
// ============================================

esp_err_t handleRootRequest(httpd_req_t* req) {
    const char* html = 
        "<!DOCTYPE html><html><head><title>Mars Rover Camera</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>"
        "body{font-family:Arial;text-align:center;margin:20px;background:#1a1a1a;color:#fff}"
        "img{max-width:100%;height:auto;border:2px solid #4CAF50;border-radius:8px}"
        "button{padding:12px 24px;margin:10px;font-size:16px;background:#4CAF50;color:#fff;"
        "border:none;border-radius:5px;cursor:pointer;transition:0.3s}"
        "button:hover{background:#45a049}"
        "button:disabled{background:#666;cursor:not-allowed}"
        "#status{padding:10px;background:#333;border-radius:5px;margin:10px auto;max-width:400px}"
        ".loading{color:#FFA500}"
        "</style></head>"
        "<body><h1>🚀 Mars Rover Camera</h1>"
        "<img id='stream' src='/stream' alt='Video Stream'>"
        "<br>"
        "<button id='captureBtn' onclick='capturePhoto()'>📷 Capture Photo</button>"
        "<button onclick='location.reload()'>🔄 Refresh</button>"
        "<p id='status'>Status: Ready</p>"
        "<script>"
        "let capturing = false;"
        "async function capturePhoto() {"
        "  if (capturing) return;"
        "  capturing = true;"
        "  const btn = document.getElementById('captureBtn');"
        "  const status = document.getElementById('status');"
        "  btn.disabled = true;"
        "  status.innerHTML = '<span class=\"loading\">📸 Capturing photo...</span>';"
        "  try {"
        "    const response = await fetch('/capture');"
        "    if (!response.ok) throw new Error('Capture failed');"
        "    const blob = await response.blob();"
        "    const url = URL.createObjectURL(blob);"
        "    const a = document.createElement('a');"
        "    a.href = url;"
        "    a.download = 'rover_photo_' + Date.now() + '.jpg';"
        "    a.click();"
        "    URL.revokeObjectURL(url);"
        "    status.innerHTML = '✅ Photo captured successfully!';"
        "    setTimeout(() => { status.innerHTML = 'Status: Ready'; }, 3000);"
        "  } catch (err) {"
        "    status.innerHTML = '❌ Error: ' + err.message;"
        "  } finally {"
        "    capturing = false;"
        "    btn.disabled = false;"
        "  }"
        "}"
        // Автоматичне оновлення статусу
        "setInterval(async () => {"
        "  try {"
        "    const r = await fetch('/status');"
        "    const data = await r.json();"
        "    if (!capturing) {"
        "      document.getElementById('status').innerHTML = "
        "        'Camera: ' + data.camera + ' | Streaming: ' + data.streaming;"
        "    }"
        "  } catch(e) {}"
        "}, 5000);"
        "</script></body></html>";
    
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, strlen(html));
}

// ОНОВЛЕНИЙ handler для capture - тепер працює зі стрімом
esp_err_t handleCaptureRequest(httpd_req_t *req) {
    ESP_LOGI(TAG, "Capture request received");
    
    // Встановлюємо флаг для стріму
    requestCaptureFromStream();
    
    // Чекаємо поки стрім зробить фото (максимум 5 секунд)
    int timeout_ms = 5000;
    int waited_ms = 0;
    while (!isCaptureReady() && waited_ms < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(50));
        waited_ms += 50;
    }
    
    if (!isCaptureReady()) {
        ESP_LOGE(TAG, "Capture timeout");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Capture timeout");
        return ESP_FAIL;
    }
    
    // Отримуємо готове фото
    photo_data_t photo = getCapturedPhoto();
    
    if (!photo.buffer) {
        ESP_LOGE(TAG, "No captured photo available");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No photo available");
        return ESP_FAIL;
    }
    
    // Відправляємо фото
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    esp_err_t res = httpd_resp_send(req, (const char*)photo.buffer, photo.length);
    
    // Звільняємо буфер
    free(photo.buffer);
    
    ESP_LOGI(TAG, "Photo sent successfully");
    return res;
}

// ОНОВЛЕНИЙ handler для стріму - тепер перевіряє флаг capture
esp_err_t handleStreamRequest(httpd_req_t* req) {
    esp_err_t res = ESP_OK;
    
    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    ESP_LOGI(TAG, "Stream started");
    
    while (streaming_active) {
        // Отримуємо frame
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed in stream");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Перевіряємо чи потрібен capture
        if (capture_request_flag) {
            ESP_LOGI(TAG, "Processing capture request in stream");
            
            if (xSemaphoreTake(capture_mutex, pdMS_TO_TICKS(100))) {
                // Звільняємо старе фото якщо є
                if (captured_photo.buffer != NULL) {
                    free(captured_photo.buffer);
                }
                
                // Зберігаємо поточний кадр
                captured_photo.buffer = (uint8_t*)malloc(fb->len);
                if (captured_photo.buffer != NULL) {
                    memcpy(captured_photo.buffer, fb->buf, fb->len);
                    captured_photo.length = fb->len;
                    capture_ready = true;
                    ESP_LOGI(TAG, "Photo captured in stream: %d bytes", captured_photo.length);
                } else {
                    ESP_LOGE(TAG, "Failed to allocate memory for capture");
                }
                
                capture_request_flag = false;
                xSemaphoreGive(capture_mutex);
            }
        }

        // Відправляємо frame для стріму
        char header[128];
        int len = snprintf(header, sizeof(header),
                           "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                           fb->len);

        if (httpd_resp_send_chunk(req, header, len) != ESP_OK) {
            esp_camera_fb_return(fb);
            break;
        }
        
        if (httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len) != ESP_OK) {
            esp_camera_fb_return(fb);
            break;
        }
        
        if (httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) {
            esp_camera_fb_return(fb);
            break;
        }

        esp_camera_fb_return(fb);
        
        // ВАЖЛИВО: Невелика затримка для можливості обробки інших запитів
        vTaskDelay(pdMS_TO_TICKS(30)); // ~33fps
    }

    ESP_LOGI(TAG, "Stream ended");
    return res;
}

esp_err_t handleStatusRequest(httpd_req_t* req) {
    char json[200];
    snprintf(json, sizeof(json),
             "{\"camera\":\"%s\",\"streaming\":\"%s\",\"fps\":30}",
             camera_initialized ? "initialized" : "not initialized",
             streaming_active ? "active" : "inactive");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, strlen(json));
}

bool initWebServer(uint16_t port) {
    if (server != NULL) {
        ESP_LOGW(TAG, "Web server already running");
        return true;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = port;
    config.ctrl_port = 32768;
    config.max_open_sockets = 7;
    config.max_uri_handlers = 8;
    config.task_priority = 5;
    config.stack_size = 8192; // Збільшено для стабільності

    ESP_LOGI(TAG, "Starting web server on port: %d", port);

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start web server");
        return false;
    }

    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = handleRootRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &root_uri);

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = handleStreamRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &stream_uri);

    httpd_uri_t capture_uri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = handleCaptureRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &capture_uri);

    httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = handleStatusRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &status_uri);

    ESP_LOGI(TAG, "Web server started successfully");
    return true;
}

void stopWebServer() {
    if (server != NULL) {
        httpd_stop(server);
        server = NULL;
        ESP_LOGI(TAG, "Web server stopped");
    }
}

httpd_handle_t getServerHandle() {
    return server;
}