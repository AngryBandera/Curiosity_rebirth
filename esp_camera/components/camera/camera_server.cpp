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
static volatile bool streaming_active = false;

// Конфігурація камери для ESP32-WROVER
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
    .jpeg_quality = 10,
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
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
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
    return "Camera ready";
}

// ============================================
// HTTP Handlers - СИНХРОННІ АЛЕ БЕЗ БЛОКУВАННЯ
// ============================================

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
        "button:hover{background:#45a049}"
        "button:disabled{background:#666;cursor:not-allowed;opacity:0.5}"
        "button.stop{background:#f44336}"
        "button.stop:hover{background:#da190b}"
        "#status{padding:15px;background:#333;border-radius:8px;margin:20px auto;"
        "max-width:500px;font-size:16px;border:2px solid #555}"
        ".active{color:#4CAF50;font-weight:bold}"
        ".inactive{color:#f44336;font-weight:bold}"
        "</style></head>"
        "<body>"
        "<h1>🚀 Mars Rover Camera</h1>"
        "<img id='stream' src='/stream' alt='Loading...'>"
        "<div class='controls'>"
        "<button id='startBtn' onclick='toggleStream(true)'>▶️ START</button>"
        "<button id='stopBtn' class='stop' onclick='toggleStream(false)'>⏹ STOP</button>"
        "</div>"
        "<div id='status'>Ready</div>"
        "<script>"
        "let statusDiv=document.getElementById('status');"
        "let startBtn=document.getElementById('startBtn');"
        "let stopBtn=document.getElementById('stopBtn');"
        
        "async function toggleStream(start){"
        "  let url=start?'/stream/start':'/stream/stop';"
        "  startBtn.disabled=true;stopBtn.disabled=true;"
        "  try{"
        "    let r=await fetch(url);"
        "    if(r.ok){"
        "      statusDiv.innerHTML=start"
        "        ?'<span class=\"active\">🎥 STREAMING</span>'"
        "        :'<span class=\"inactive\">⏸ STOPPED</span>';"
        "      console.log(start?'Started':'Stopped');"
        "    }"
        "  }catch(e){console.error(e);}"
        "  startBtn.disabled=false;stopBtn.disabled=false;"
        "}"
        
        "setInterval(async()=>{"
        "  try{"
        "    let r=await fetch('/status');"
        "    let d=await r.json();"
        "    statusDiv.innerHTML=d.streaming"
        "      ?'<span class=\"active\">🎥 STREAMING</span>'"
        "      :'<span class=\"inactive\">⏸ STOPPED</span>';"
        "  }catch(e){}"
        "},2000);"
        "</script>"
        "</body></html>";
    
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, strlen(html));
}

// ⭐ КРИТИЧНА ЗМІНА: Стрім з правильним використанням httpd_socket_send
esp_err_t handleStreamRequest(httpd_req_t* req) {
    ESP_LOGI(TAG, "📹 Stream connected");
    
    if (!camera_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    camera_fb_t* fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    
    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "10");

    // ⭐ Отримуємо handle сервера та socket fd
    httpd_handle_t hd = server;  // Глобальна змінна сервера
    int fd = httpd_req_to_sockfd(req);
    
    while (true) {
        // Перевірка чи клієнт ще підключений
        if (fd < 0) {
            ESP_LOGW(TAG, "Invalid socket");
            break;
        }
        
        if (streaming_active) {
            fb = esp_camera_fb_get();
            if (!fb) {
                ESP_LOGW(TAG, "Frame get failed");
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;

            char part_buf[128];
            size_t hlen = snprintf(part_buf, sizeof(part_buf), 
                "--frame\r\n"
                "Content-Type: image/jpeg\r\n"
                "Content-Length: %u\r\n"
                "\r\n", 
                _jpg_buf_len);
            
            // ⭐ ПРАВИЛЬНА сигнатура: httpd_socket_send(handle, sockfd, buf, len, flags)
            // Відправляємо заголовок
            if (httpd_socket_send(hd, fd, part_buf, hlen, 0) < 0) {
                esp_camera_fb_return(fb);
                ESP_LOGI(TAG, "Client disconnected (header)");
                break;
            }

            // Відправляємо JPEG
            if (httpd_socket_send(hd, fd, (const char *)_jpg_buf, _jpg_buf_len, 0) < 0) {
                esp_camera_fb_return(fb);
                ESP_LOGI(TAG, "Client disconnected (body)");
                break;
            }

            // Відправляємо boundary
            if (httpd_socket_send(hd, fd, "\r\n", 2, 0) < 0) {
                esp_camera_fb_return(fb);
                ESP_LOGI(TAG, "Client disconnected (boundary)");
                break;
            }

            esp_camera_fb_return(fb);
            fb = NULL;
            
            // 10 FPS
            vTaskDelay(pdMS_TO_TICKS(100));
            
        } else {
            // Placeholder коли стрім зупинено
            const char* svg = 
                "<svg xmlns='http://www.w3.org/2000/svg' width='800' height='600'>"
                "<rect width='100%' height='100%' fill='#000'/>"
                "<text x='50%' y='50%' font-size='28' fill='#f44' text-anchor='middle'>"
                "Stream Stopped</text></svg>";
            
            char buf[256];
            size_t len = snprintf(buf, sizeof(buf), 
                "--frame\r\n"
                "Content-Type: image/svg+xml\r\n"
                "Content-Length: %d\r\n"
                "\r\n%s\r\n",
                (int)strlen(svg), svg);
            
            if (httpd_socket_send(hd, fd, buf, len, 0) < 0) {
                break;
            }
            
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        // ⭐ ВАЖЛИВО: Даємо можливість іншим задачам працювати
        taskYIELD();
    }

    ESP_LOGI(TAG, "🔴 Stream ended");
    return res;
}

esp_err_t handleStartStreamRequest(httpd_req_t* req) {
    ESP_LOGI(TAG, "🟢 START requested");
    
    startVideoStream();
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    const char* body = "{\"status\":\"ok\"}";
    return httpd_resp_send(req, body, strlen(body));
}

esp_err_t handleStopStreamRequest(httpd_req_t* req) {
    ESP_LOGI(TAG, "🔴 STOP requested");
    
    stopVideoStream();
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    const char* body = "{\"status\":\"ok\"}";
    return httpd_resp_send(req, body, strlen(body));
}

esp_err_t handleStatusRequest(httpd_req_t* req) {
    char json[128];
    snprintf(json, sizeof(json),
             "{\"streaming\":%s,\"camera\":\"%s\"}",
             streaming_active ? "true" : "false",
             camera_initialized ? "ready" : "not_ready");
    
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
    
    // ⭐ КРИТИЧНІ налаштування для роботи зі стрімом
    config.max_open_sockets = 7;
    config.max_uri_handlers = 8;
    config.stack_size = 8192;           // Збільшений стек
    config.task_priority = 5;
    config.core_id = 1;
    
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;
    config.backlog_conn = 5;
    
    // ⭐ Важливо для стріму
    config.global_user_ctx = NULL;
    config.global_user_ctx_free_fn = NULL;
    config.global_transport_ctx = NULL;
    config.global_transport_ctx_free_fn = NULL;
    config.open_fn = NULL;
    config.close_fn = NULL;

    ESP_LOGI(TAG, "Starting web server...");
    
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start web server");
        return false;
    }

    // Реєструємо handlers
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

    httpd_uri_t uri_start = {
        .uri = "/stream/start",
        .method = HTTP_GET,
        .handler = handleStartStreamRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &uri_start);

    httpd_uri_t uri_stop = {
        .uri = "/stream/stop",
        .method = HTTP_GET,
        .handler = handleStopStreamRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &uri_stop);

    httpd_uri_t uri_status = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = handleStatusRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &uri_status);

    ESP_LOGI(TAG, "✅ Web server started on port %d", port);
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