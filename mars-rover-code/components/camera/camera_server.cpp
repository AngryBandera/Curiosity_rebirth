#include "camera_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include <string.h>

static const char* TAG = "CAMERA_SERVER";

// Глобальні змінні
static httpd_handle_t server = NULL;
static bool camera_initialized = false;
static bool streaming_active = false;

// Конфігурація пінів для ESP32-WROVER з OV2640
// ВАЖЛИВО: Перевір pinout своєї конкретної плати!
static camera_config_t camera_config = {
    .pin_pwdn = -1,        // WROVER зазвичай не має power down
    .pin_reset = -1,       // Software reset
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
    .frame_size = FRAMESIZE_SVGA,    // 800x600, можна FRAMESIZE_XGA для WROVER
    .jpeg_quality = 12,               // 0-63, менше = краща якість
    .fb_count = 2,                    // WROVER має багато RAM, можна 2-3
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
        // Налаштування для кращого відео
        s->set_brightness(s, 0);     // -2 to 2
        s->set_contrast(s, 0);       // -2 to 2
        s->set_saturation(s, 0);     // -2 to 2
        s->set_special_effect(s, 0); // 0 = No Effect
        s->set_whitebal(s, 1);       // 0 = disable, 1 = enable
        s->set_awb_gain(s, 1);       // 0 = disable, 1 = enable
        s->set_wb_mode(s, 0);        // 0 to 4
        s->set_exposure_ctrl(s, 1);  // 0 = disable, 1 = enable
        s->set_aec2(s, 0);           // 0 = disable, 1 = enable
        s->set_ae_level(s, 0);       // -2 to 2
        s->set_aec_value(s, 300);    // 0 to 1200
        s->set_gain_ctrl(s, 1);      // 0 = disable, 1 = enable
        s->set_agc_gain(s, 0);       // 0 to 30
        s->set_gainceiling(s, (gainceiling_t)0); // 0 to 6
        s->set_bpc(s, 0);            // 0 = disable, 1 = enable
        s->set_wpc(s, 1);            // 0 = disable, 1 = enable
        s->set_raw_gma(s, 1);        // 0 = disable, 1 = enable
        s->set_lenc(s, 1);           // 0 = disable, 1 = enable
        s->set_hmirror(s, 0);        // 0 = disable, 1 = enable
        s->set_vflip(s, 0);          // 0 = disable, 1 = enable
        s->set_dcw(s, 1);            // 0 = disable, 1 = enable
        s->set_colorbar(s, 0);       // 0 = disable, 1 = enable
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

    // Захоплюємо frame
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        return photo;
    }

    // Копіюємо дані (щоб можна було звільнити fb)
    photo.buffer = (uint8_t*)malloc(fb->len);
    if (photo.buffer != NULL) {
        memcpy(photo.buffer, fb->buf, fb->len);
        photo.length = fb->len;
        ESP_LOGI(TAG, "Photo captured: %d bytes", photo.length);
    } else {
        ESP_LOGE(TAG, "Failed to allocate memory for photo");
    }

    // Звільняємо frame buffer
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
// Функції для вебсервера
// ============================================

// Handler для root сторінки
esp_err_t handleRootRequest(httpd_req_t* req) {
    const char* html = 
        "<!DOCTYPE html><html><head><title>Mars Rover Camera</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>body{font-family:Arial;text-align:center;margin:20px}"
        "img{max-width:100%;height:auto;border:2px solid #333}"
        "button{padding:10px 20px;margin:10px;font-size:16px}</style></head>"
        "<body><h1>Mars Rover Camera</h1>"
        "<img id='stream' src='/stream'>"
        "<br><button onclick='capture()'>Capture Photo</button>"
        "<button onclick='location.reload()'>Refresh</button>"
        "<p id='status'>Status: Ready</p>"
        "<script>"
        "function capture(){fetch('/capture').then(r=>r.blob()).then(b=>{"
        "let url=URL.createObjectURL(b);let a=document.createElement('a');"
        "a.href=url;a.download='photo.jpg';a.click();})}"
        "</script></body></html>";
    
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, strlen(html));
}

// Handler для MJPEG стріму
esp_err_t handleStreamRequest(httpd_req_t* req) {
    camera_fb_t* fb = NULL;
    esp_err_t res = ESP_OK;
    
    // Встановлюємо заголовки для MJPEG
    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");

    while (streaming_active) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        // Відправляємо JPEG frame
        char part_buf[64];
        snprintf(part_buf, sizeof(part_buf), 
                 "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                 fb->len);
        
        if (httpd_resp_send_chunk(req, part_buf, strlen(part_buf)) != ESP_OK) {
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
        fb = NULL;
    }

    return res;
}

// Handler для capture одного фото
esp_err_t handlePhotoRequest(httpd_req_t* req) {
    photo_data_t photo = capturePhoto();
    
    if (photo.buffer == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    
    esp_err_t res = httpd_resp_send(req, (const char*)photo.buffer, photo.length);
    
    releasePhotoBuffer(photo);
    return res;
}

// Handler для статусу
esp_err_t handleStatusRequest(httpd_req_t* req) {
    char json[200];
    snprintf(json, sizeof(json),
             "{\"camera\":\"%s\",\"streaming\":%s,\"fps\":15}",
             camera_initialized ? "initialized" : "not initialized",
             streaming_active ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, strlen(json));
}

// Ініціалізація вебсервера
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
    config.stack_size = 4096;

    ESP_LOGI(TAG, "Starting web server on port: %d", port);

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start web server");
        return false;
    }

    // Реєструємо URI handlers
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
        .handler = handlePhotoRequest,
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