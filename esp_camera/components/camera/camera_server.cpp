#include "camera_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char* TAG = "CAMERA_SERVER";

// Глобальні змінні
static httpd_handle_t server = NULL;
static bool camera_initialized = false;
static bool streaming_active = false;

// Система для capture
static volatile bool capture_request_flag = false;
static photo_data_t captured_photo = {NULL, 0};
static volatile bool capture_ready = false;
static SemaphoreHandle_t capture_mutex = NULL;

// Для стріму через таск
static TaskHandle_t stream_task_handle = NULL;
static QueueHandle_t stream_client_queue = NULL;
typedef struct {
    httpd_req_t* req;
    bool active;
} stream_client_t;

static stream_client_t active_stream_client = {NULL, false};
static SemaphoreHandle_t stream_client_mutex = NULL;

// Конфігурація камери
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
// Окремий таск для стріму
// ============================================

void stream_task(void* pvParameters) {
    ESP_LOGI(TAG, "Stream task started");
    
    uint32_t frame_count = 0;
    uint32_t loop_count = 0;
    
    while (true) {
        loop_count++;
        
        // Детальне логування кожні 50 ітерацій
        if (loop_count % 50 == 0) {
            ESP_LOGI(TAG, "Task alive: streaming=%d, client=%d", 
                     streaming_active, active_stream_client.active);
        }
        
        // Перевіряємо чи є активний клієнт
        httpd_req_t* req = NULL;
        bool has_client = false;
        
        if (xSemaphoreTake(stream_client_mutex, pdMS_TO_TICKS(10))) {
            if (active_stream_client.active && active_stream_client.req != NULL) {
                req = active_stream_client.req;
                has_client = true;
            }
            xSemaphoreGive(stream_client_mutex);
        }
        
        if (!has_client) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        if (!streaming_active) {
            ESP_LOGW(TAG, "Client connected but streaming disabled!");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Перший кадр - логуємо
        if (frame_count == 0) {
            ESP_LOGI(TAG, "🎬 Starting to send frames");
        }
        
        // Отримуємо кадр
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Failed to get frame");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Логуємо перший кадр
        if (frame_count == 0) {
            ESP_LOGI(TAG, "✅ Got first frame: %d bytes", fb->len);
        }
        
        // Перевіряємо capture запит
        if (capture_request_flag) {
            ESP_LOGI(TAG, "📸 Processing capture request (frame %lu)", frame_count);
            
            if (xSemaphoreTake(capture_mutex, pdMS_TO_TICKS(50))) {
                if (captured_photo.buffer != NULL) {
                    free(captured_photo.buffer);
                    captured_photo.buffer = NULL;
                }
                
                captured_photo.buffer = (uint8_t*)malloc(fb->len);
                if (captured_photo.buffer != NULL) {
                    memcpy(captured_photo.buffer, fb->buf, fb->len);
                    captured_photo.length = fb->len;
                    capture_ready = true;
                    capture_request_flag = false;
                    ESP_LOGI(TAG, "✅ Photo saved: %d bytes", captured_photo.length);
                } else {
                    ESP_LOGE(TAG, "❌ Failed to allocate memory");
                    capture_request_flag = false;
                }
                xSemaphoreGive(capture_mutex);
            }
        }
        
        // Відправляємо кадр
        char header[128];
        int len = snprintf(header, sizeof(header),
                          "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                          fb->len);
        
        esp_err_t res = ESP_OK;
        
        res = httpd_resp_send_chunk(req, header, len);
        if (res != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send header: %d", res);
            esp_camera_fb_return(fb);
            if (xSemaphoreTake(stream_client_mutex, pdMS_TO_TICKS(100))) {
                active_stream_client.active = false;
                active_stream_client.req = NULL;
                xSemaphoreGive(stream_client_mutex);
            }
            frame_count = 0;
            continue;
        }
        
        res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
        if (res != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send data: %d", res);
            esp_camera_fb_return(fb);
            if (xSemaphoreTake(stream_client_mutex, pdMS_TO_TICKS(100))) {
                active_stream_client.active = false;
                active_stream_client.req = NULL;
                xSemaphoreGive(stream_client_mutex);
            }
            frame_count = 0;
            continue;
        }
        
        res = httpd_resp_send_chunk(req, "\r\n", 2);
        if (res != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send footer: %d", res);
            esp_camera_fb_return(fb);
            if (xSemaphoreTake(stream_client_mutex, pdMS_TO_TICKS(100))) {
                active_stream_client.active = false;
                active_stream_client.req = NULL;
                xSemaphoreGive(stream_client_mutex);
            }
            frame_count = 0;
            continue;
        }
        
        esp_camera_fb_return(fb);
        frame_count++;
        
        // Логуємо кожні 100 кадрів
        if (frame_count % 100 == 0) {
            ESP_LOGI(TAG, "📺 Sent %lu frames", frame_count);
        }
        
        // Затримка для ~25 FPS
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

// ============================================
// Функції для камери
// ============================================

bool initCamera(const camera_config_params_t* config) {
    if (camera_initialized) {
        ESP_LOGW(TAG, "Camera already initialized");
        return true;
    }

    // Створюємо мютекси
    capture_mutex = xSemaphoreCreateMutex();
    stream_client_mutex = xSemaphoreCreateMutex();
    if (!capture_mutex || !stream_client_mutex) {
        ESP_LOGE(TAG, "Failed to create mutexes");
        return false;
    }

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

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        return false;
    }

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
    ESP_LOGI(TAG, "✅ Camera initialized");
    
    // Запускаємо таск для стріму
    xTaskCreate(stream_task, "stream_task", 4096, NULL, 5, &stream_task_handle);
    ESP_LOGI(TAG, "✅ Stream task created");
    
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
    ESP_LOGI(TAG, "🎥 Streaming enabled");
    return true;
}

bool stopVideoStream() {
    streaming_active = false;
    ESP_LOGI(TAG, "🛑 Streaming disabled");
    return true;
}

bool isStreaming() {
    return streaming_active;
}

bool isCameraInitialized() {
    return camera_initialized;
}

const char* getCameraStatus() {
    if (!camera_initialized) return "Not initialized";
    if (streaming_active) return "Streaming active";
    return "Ready";
}

void requestCaptureFromStream() {
    capture_request_flag = true;
    capture_ready = false;
    ESP_LOGI(TAG, "🔔 Capture requested");
}

bool isCaptureReady() {
    return capture_ready;
}

photo_data_t getCapturedPhoto() {
    photo_data_t photo = {NULL, 0};
    
    if (xSemaphoreTake(capture_mutex, pdMS_TO_TICKS(100))) {
        if (capture_ready && captured_photo.buffer != NULL) {
            photo = captured_photo;
            captured_photo.buffer = NULL;
            captured_photo.length = 0;
            capture_ready = false;
        }
        xSemaphoreGive(capture_mutex);
    }
    return photo;
}

// ============================================
// HTTP Handlers
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
        "  status.innerHTML = '<span class=\"loading\">📸 Capturing...</span>';"
        "  try {"
        "    const response = await fetch('/capture');"
        "    if (!response.ok) throw new Error('Failed');"
        "    const blob = await response.blob();"
        "    const url = URL.createObjectURL(blob);"
        "    const a = document.createElement('a');"
        "    a.href = url;"
        "    a.download = 'rover_' + Date.now() + '.jpg';"
        "    a.click();"
        "    URL.revokeObjectURL(url);"
        "    status.innerHTML = '✅ Photo saved!';"
        "    setTimeout(() => { status.innerHTML = 'Status: Ready'; }, 2000);"
        "  } catch (err) {"
        "    status.innerHTML = '❌ Error: ' + err.message;"
        "  } finally {"
        "    capturing = false;"
        "    btn.disabled = false;"
        "  }"
        "}"
        "</script></body></html>";
    
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, strlen(html));
}

esp_err_t handleStreamRequest(httpd_req_t* req) {
    ESP_LOGI(TAG, "🎬 Stream client connected");
    
    // ВАЖЛИВО: Спочатку відправляємо headers
    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    
    // Відправляємо порожній chunk щоб встановити з'єднання
    esp_err_t res = httpd_resp_send_chunk(req, "", 0);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to establish stream connection");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Stream headers sent, registering client");
    
    // Реєструємо клієнта
    if (xSemaphoreTake(stream_client_mutex, pdMS_TO_TICKS(1000))) {
        // Якщо вже є клієнт - відключаємо його
        if (active_stream_client.active) {
            ESP_LOGW(TAG, "Replacing existing stream client");
        }
        active_stream_client.req = req;
        active_stream_client.active = true;
        xSemaphoreGive(stream_client_mutex);
        ESP_LOGI(TAG, "✅ Client registered");
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Чекаємо поки таск обслуговує клієнта
    while (true) {
        bool still_active = false;
        
        if (xSemaphoreTake(stream_client_mutex, pdMS_TO_TICKS(100))) {
            still_active = active_stream_client.active;
            xSemaphoreGive(stream_client_mutex);
        }
        
        if (!still_active) {
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "📴 Stream client disconnected");
    return ESP_OK;
}

esp_err_t handleCaptureRequest(httpd_req_t *req) {
    ESP_LOGI(TAG, "📷 Capture request received");
    
    requestCaptureFromStream();
    
    // Чекаємо готове фото
    int timeout_ms = 3000;
    int waited_ms = 0;
    while (!isCaptureReady() && waited_ms < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(10));
        waited_ms += 10;
    }
    
    if (!isCaptureReady()) {
        ESP_LOGE(TAG, "❌ Capture timeout");
        capture_request_flag = false;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Timeout");
        return ESP_FAIL;
    }
    
    photo_data_t photo = getCapturedPhoto();
    
    if (!photo.buffer || photo.length == 0) {
        ESP_LOGE(TAG, "❌ No photo data");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No data");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "📤 Sending photo: %d bytes", photo.length);
    
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    esp_err_t res = httpd_resp_send(req, (const char*)photo.buffer, photo.length);
    
    free(photo.buffer);
    
    if (res == ESP_OK) {
        ESP_LOGI(TAG, "✅ Photo sent successfully");
    }
    
    return res;
}

esp_err_t handleStatusRequest(httpd_req_t* req) {
    char json[256];
    snprintf(json, sizeof(json),
             "{\"camera\":\"%s\",\"streaming\":\"%s\",\"fps\":25}",
             camera_initialized ? "initialized" : "not initialized",
             streaming_active ? "active" : "inactive");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, strlen(json));
}

bool initWebServer(uint16_t port) {
    if (server != NULL) {
        ESP_LOGW(TAG, "Server already running");
        return true;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = port;
    config.ctrl_port = 32768;
    config.max_open_sockets = 7;
    config.max_uri_handlers = 8;
    config.task_priority = 5;
    config.stack_size = 8192;
    config.lru_purge_enable = true;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start server");
        return false;
    }

    httpd_uri_t root_uri = {.uri = "/", .method = HTTP_GET, .handler = handleRootRequest, .user_ctx = NULL};
    httpd_uri_t stream_uri = {.uri = "/stream", .method = HTTP_GET, .handler = handleStreamRequest, .user_ctx = NULL};
    httpd_uri_t capture_uri = {.uri = "/capture", .method = HTTP_GET, .handler = handleCaptureRequest, .user_ctx = NULL};
    httpd_uri_t status_uri = {.uri = "/status", .method = HTTP_GET, .handler = handleStatusRequest, .user_ctx = NULL};

    httpd_register_uri_handler(server, &root_uri);
    httpd_register_uri_handler(server, &stream_uri);
    httpd_register_uri_handler(server, &capture_uri);
    httpd_register_uri_handler(server, &status_uri);

    ESP_LOGI(TAG, "✅ Web server started on port %d", port);
    return true;
}

void stopWebServer() {
    if (server != NULL) {
        httpd_stop(server);
        server = NULL;
        ESP_LOGI(TAG, "Server stopped");
    }
}

httpd_handle_t getServerHandle() {
    return server;
}