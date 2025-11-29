#include "camera_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "CAMERA_SERVER";

// Глобальні змінні
static httpd_handle_t server = NULL;
static bool camera_initialized = false;
static bool streaming_active = false;

// Конфігурація камери для ESP32-WROVER
static camera_config_t camera_config = {
    .pin_pwdn = -1,        // WROVER не має power down
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
    .frame_size = FRAMESIZE_SVGA,    // 800x600
    .jpeg_quality = 12,               // 0-63, менше = краща якість
    .fb_count = 2,                    // 2 буфери для плавного стріму
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

    // Налаштування сенсора для кращого відео
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
    ESP_LOGI(TAG, "✅ Camera initialized successfully");
    return true;
}

bool startVideoStream() {
    if (!camera_initialized) {
        ESP_LOGE(TAG, "Cannot start stream - camera not initialized");
        return false;
    }
    
    streaming_active = true;
    ESP_LOGI(TAG, "🎥 Video streaming STARTED");
    return true;
}

bool stopVideoStream() {
    streaming_active = false;
    ESP_LOGI(TAG, "🛑 Video streaming STOPPED");
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
    return "Camera ready (stream stopped)";
}

// ============================================
// HTTP Handlers
// ============================================

// Головна сторінка з вбудованим стрімом
esp_err_t handleRootRequest(httpd_req_t* req) {
    const char* html = 
        "<!DOCTYPE html><html><head><title>Mars Rover Camera</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>"
        "body{font-family:Arial;text-align:center;margin:20px;background:#1a1a1a;color:#fff}"
        "h1{color:#4CAF50}"
        "img{max-width:100%;height:auto;border:3px solid #4CAF50;border-radius:8px;"
        "background:#000;box-shadow:0 4px 8px rgba(0,0,0,0.5)}"
        ".controls{margin:20px 0}"
        "button{padding:15px 30px;margin:10px;font-size:18px;background:#4CAF50;color:#fff;"
        "border:none;border-radius:8px;cursor:pointer;transition:all 0.3s;font-weight:bold}"
        "button:hover{background:#45a049;transform:scale(1.05)}"
        "button:active{transform:scale(0.95)}"
        "button.stop{background:#f44336}"
        "button.stop:hover{background:#da190b}"
        "#status{padding:15px;background:#333;border-radius:8px;margin:20px auto;"
        "max-width:500px;font-size:16px;border:2px solid #555}"
        ".active{color:#4CAF50;font-weight:bold}"
        ".inactive{color:#f44336;font-weight:bold}"
        "</style></head>"
        "<body>"
        "<h1>🚀 Mars Rover Camera Stream</h1>"
        "<img id='stream' src='/stream' alt='Connecting to stream...'>"
        "<div class='controls'>"
        "<button id='startBtn' onclick='startStream()'>▶️ START STREAM</button>"
        "<button id='stopBtn' class='stop' onclick='stopStream()'>⏹ STOP STREAM</button>"
        "</div>"
        "<div id='status'>Loading status...</div>"
        "<script>"
        "let streamImg = document.getElementById('stream');"
        "let statusDiv = document.getElementById('status');"
        
        // Функція запуску стріму
        "async function startStream(){"
        "  try{"
        "    const response = await fetch('/stream/start');"
        "    if(response.ok){"
        "      statusDiv.innerHTML='<span class=\"active\">🎥 Streaming ACTIVE</span>';"
        "      streamImg.src='/stream?t='+Date.now();"  // перезавантажуємо стрім
        "    }"
        "  }catch(e){"
        "    statusDiv.innerHTML='<span class=\"inactive\">❌ Error: '+e.message+'</span>';"
        "  }"
        "}"
        
        // Функція зупинки стріму
        "async function stopStream(){"
        "  try{"
        "    const response = await fetch('/stream/stop');"
        "    if(response.ok){"
        "      statusDiv.innerHTML='<span class=\"inactive\">⏸ Streaming STOPPED</span>';"
        "      streamImg.src='/stream?t='+Date.now();"  // перезавантажуємо стрім
        "    }"
        "  }catch(e){"
        "    statusDiv.innerHTML='<span class=\"inactive\">❌ Error: '+e.message+'</span>';"
        "  }"
        "}"
        
        // Оновлення статусу кожні 2 секунди
        "async function updateStatus(){"
        "  try{"
        "    const response = await fetch('/status');"
        "    const data = await response.json();"
        "    if(data.streaming){"
        "      statusDiv.innerHTML='<span class=\"active\">🎥 Streaming ACTIVE</span>';"
        "    }else{"
        "      statusDiv.innerHTML='<span class=\"inactive\">⏸ Streaming STOPPED</span>';"
        "    }"
        "  }catch(e){}"
        "}"
        
        // Перевірка статусу при завантаженні
        "updateStatus();"
        "setInterval(updateStatus, 2000);"
        "</script>"
        "</body></html>";
    
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, strlen(html));
}

// Handler для MJPEG відео стріму
esp_err_t handleStreamRequest(httpd_req_t* req) {
    ESP_LOGI(TAG, "📹 Stream request received");
    
    if (!camera_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    camera_fb_t* fb = NULL;
    esp_err_t res = ESP_OK;
    
    // Встановлюємо заголовки для MJPEG стріму
    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // Головний цикл стрімінгу
    while (true) {
        // Якщо стрімінг активний - відправляємо реальні кадри
        if (streaming_active) {
            // Отримуємо кадр з камери
            fb = esp_camera_fb_get();
            if (!fb) {
                ESP_LOGE(TAG, "Camera capture failed");
                res = ESP_FAIL;
                break;
            }

            // Формуємо заголовок для MJPEG frame
            char part_buf[64];
            snprintf(part_buf, sizeof(part_buf), 
                     "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                     fb->len);
            
            // Відправляємо заголовок
            if (httpd_resp_send_chunk(req, part_buf, strlen(part_buf)) != ESP_OK) {
                esp_camera_fb_return(fb);
                ESP_LOGI(TAG, "Client disconnected");
                break;
            }

            // Відправляємо JPEG дані
            if (httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len) != ESP_OK) {
                esp_camera_fb_return(fb);
                ESP_LOGI(TAG, "Client disconnected");
                break;
            }

            // Відправляємо закінчення frame
            if (httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) {
                esp_camera_fb_return(fb);
                ESP_LOGI(TAG, "Client disconnected");
                break;
            }

            // Звільняємо frame buffer
            esp_camera_fb_return(fb);
            fb = NULL;
            
            // Невелика затримка для ~10 FPS
            vTaskDelay(pdMS_TO_TICKS(100));
            
        } else {
            // Якщо стрімінг неактивний - показуємо чорний екран з текстом
            const char* placeholder = 
                "<svg xmlns='http://www.w3.org/2000/svg' width='800' height='600'>"
                "<rect width='100%' height='100%' fill='#000'/>"
                "<text x='50%' y='50%' font-family='Arial' font-size='32' fill='#f44336' "
                "text-anchor='middle' dominant-baseline='middle'>"
                "⏸ Stream Stopped</text>"
                "<text x='50%' y='60%' font-family='Arial' font-size='18' fill='#999' "
                "text-anchor='middle' dominant-baseline='middle'>"
                "Press START STREAM to begin</text>"
                "</svg>";
            
            char part_buf[128];
            snprintf(part_buf, sizeof(part_buf), 
                     "--frame\r\nContent-Type: image/svg+xml\r\nContent-Length: %d\r\n\r\n",
                     (int)strlen(placeholder));
            
            if (httpd_resp_send_chunk(req, part_buf, strlen(part_buf)) != ESP_OK) {
                break;
            }
            if (httpd_resp_send_chunk(req, placeholder, strlen(placeholder)) != ESP_OK) {
                break;
            }
            if (httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) {
                break;
            }
            
            // Довша затримка коли стрім вимкнений
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    // Завершуємо multipart response
    httpd_resp_send_chunk(req, NULL, 0);
    ESP_LOGI(TAG, "🔴 Stream handler exiting");
    return res;
}

// Handler для запуску стріму
esp_err_t handleStartStreamRequest(httpd_req_t* req) {
    ESP_LOGI(TAG, "🟢 Start stream API called");
    
    if (startVideoStream()) {
        httpd_resp_set_type(req, "application/json");
        const char* body = "{\"status\":\"started\",\"streaming\":true}";
        return httpd_resp_send(req, body, strlen(body));
    } else {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
}

// Handler для зупинки стріму
esp_err_t handleStopStreamRequest(httpd_req_t* req) {
    ESP_LOGI(TAG, "🔴 Stop stream API called");
    
    stopVideoStream();
    httpd_resp_set_type(req, "application/json");
    const char* body = "{\"status\":\"stopped\",\"streaming\":false}";
    return httpd_resp_send(req, body, strlen(body));
}

// Handler для статусу
esp_err_t handleStatusRequest(httpd_req_t* req) {
    char json[256];
    snprintf(json, sizeof(json),
             "{\"camera\":\"%s\",\"streaming\":%s,\"status\":\"%s\"}",
             camera_initialized ? "initialized" : "not_initialized",
             streaming_active ? "true" : "false",
             getCameraStatus());
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, strlen(json));
}

// ============================================
// Ініціалізація вебсервера
// ============================================

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

    // Реєструємо всі endpoints
    httpd_uri_t uri_root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = handleRootRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &uri_root);

    httpd_uri_t uri_stream = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = handleStreamRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &uri_stream);

    httpd_uri_t uri_stream_start = {
        .uri = "/stream/start",
        .method = HTTP_GET,
        .handler = handleStartStreamRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &uri_stream_start);

    httpd_uri_t uri_stream_stop = {
        .uri = "/stream/stop",
        .method = HTTP_GET,
        .handler = handleStopStreamRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &uri_stream_stop);

    httpd_uri_t uri_status = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = handleStatusRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &uri_status);

    ESP_LOGI(TAG, "✅ Web server started successfully");
    ESP_LOGI(TAG, "   📍 Root page: http://[IP]/");
    ESP_LOGI(TAG, "   📹 Stream: http://[IP]/stream");
    ESP_LOGI(TAG, "   ▶️  Start: http://[IP]/stream/start");
    ESP_LOGI(TAG, "   ⏹  Stop: http://[IP]/stream/stop");
    ESP_LOGI(TAG, "   📊 Status: http://[IP]/status");
    
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