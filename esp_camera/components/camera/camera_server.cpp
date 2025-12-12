#include "camera_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "CAMERA_SERVER";

static httpd_handle_t control_server = NULL;  // Порт 80 - кнопки
static httpd_handle_t stream_server = NULL;   // Порт 81 - стрім

static bool camera_initialized = false;
static volatile bool streaming_active = false;

static SemaphoreHandle_t camera_mutex = NULL;

// MJPEG boundary
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

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
    .frame_size = FRAMESIZE_HD,
    .jpeg_quality = 12,
    .fb_count = 2,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY
};

// Функції для камери

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

    // Створюємо мьютекс для захисту доступу до камери
    camera_mutex = xSemaphoreCreateMutex();
    if (camera_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create camera mutex");
        return false;
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

// HTTP Handlers

esp_err_t handleRootRequest(httpd_req_t* req) {
    const char* html = 
        "<!DOCTYPE html><html><head><title>Mars Rover Camera</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>"
        "body{font-family:Arial;text-align:center;margin:20px;background:#1a1a1a;color:#fff}"
        "h1{color:#4CAF50}"
        "#streamContainer{max-width:100%;height:auto;border:3px solid #4CAF50;border-radius:8px;"
        "background:#000;box-shadow:0 4px 8px rgba(0,0,0,0.5);min-height:400px;display:flex;"
        "align-items:center;justify-content:center;color:#666;font-size:18px;overflow:hidden}"
        "img{max-width:100%;height:auto;display:block;margin:0 auto}"
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
        ".info{font-size:12px;color:#888;margin-top:10px}"
        "</style></head>"
        "<body>"
        "<h1>Mars Rover Camera</h1>"
        "<div id='streamContainer'>Press START to begin streaming</div>"
        "<div class='controls'>"
        "<button id='startBtn' onclick='startStream()'>START</button>"
        "<button id='stopBtn' class='stop' onclick='stopStream()' disabled>STOP</button>"
        "<button id='captureBtn' onclick='capturePhoto()'>PHOTO</button>"
        "</div>"
        "<div id='status'>Ready</div>"
        "<div class='info'>Control: Port 80 | Stream: Port 81</div>"
        "<script>"
        "let container=document.getElementById('streamContainer');"
        "let statusDiv=document.getElementById('status');"
        "let startBtn=document.getElementById('startBtn');"
        "let stopBtn=document.getElementById('stopBtn');"
        "let captureBtn=document.getElementById('captureBtn');"
        "let streamImg=null;"
        "let isStreaming=false;"
        
        "async function startStream(){"
        "  if(isStreaming)return;"
        "  startBtn.disabled=true;"
        "  console.log('Starting stream...');"
        "  try{"
        "    let r=await fetch('/stream/start');"
        "    let data=await r.json();"
        "    if(r.ok && data.status=='ok'){"
        "      isStreaming=true;"
        "      container.innerHTML='';"
        "      streamImg=document.createElement('img');"
        "      let streamPort=parseInt(window.location.port)||80;"
        "      streamPort+=1;"
        
        // Додаємо timestamp щоб обійти кеш браузера при перезапуску
        "      streamImg.src='http://'+window.location.hostname+':'+streamPort+'/stream?t=' + Date.now();"
        
        "      streamImg.style.width='100%';"
        "      streamImg.onerror=()=>{console.error('Stream error');statusDiv.innerHTML='<span class=\"inactive\">Stream Connection Lost</span>';};"
        "      streamImg.onload=()=>{console.log('Stream loaded!');};"
        "      container.appendChild(streamImg);"
        "      statusDiv.innerHTML='<span class=\"active\">STREAMING</span>';"
        "      stopBtn.disabled=false;"
        "    }else{"
        "      startBtn.disabled=false;"
        "    }"
        "  }catch(e){"
        "    startBtn.disabled=false;"
        "  }"
        "}"
        
        "async function stopStream(){"
        "  if(!isStreaming)return;"
        "  stopBtn.disabled=true;"
        "  try{"
        "    let r=await fetch('/stream/stop');"
        "    if(r.ok){"
        "      isStreaming=false;"
        "      container.innerHTML='Stream stopped - Press START to resume';"
        "      statusDiv.innerHTML='<span class=\"inactive\">⏸ STOPPED</span>';"
        "      startBtn.disabled=false;"
        "    }"
        "  }catch(e){"
        "    stopBtn.disabled=false;"
        "  }"
        "}"
        
        "async function capturePhoto(){"
        "  let wasStreaming = isStreaming;" 
        "  captureBtn.disabled=true;"
        "  statusDiv.innerHTML='<span class=\"active\">Capturing...</span>';"
        "  try{"
        "    isStreaming = false;" 
        "    let timestamp=Date.now();"
        "    let r=await fetch('/capture?t='+timestamp);"
        "    if(r.ok){"
        "      let blob=await r.blob();"
        "      let url=URL.createObjectURL(blob);"
        "      let img=document.createElement('img');"
        "      img.src=url;"
        "      img.style.width='100%';"
        "      container.innerHTML='';"
        "      container.appendChild(img);"
        "      statusDiv.innerHTML='<span class=\"active\">Photo captured! Resuming in 2s...</span>';"
        
        "      let link=document.createElement('a');"
        "      link.href=url;"
        "      link.download='capture_'+timestamp+'.jpg';"
        "      link.click();"
        
        // ЛОГІКА ПЕРЕЗАПУСКУ
        "      if(wasStreaming) {"
        "         setTimeout(() => {"
        "            startStream();"
        "         }, 2000);"
        "      } else {"
        "         startBtn.disabled=false;"
        "      }"
        
        "    }else{"
        "      statusDiv.innerHTML='<span class=\"inactive\">Capture failed</span>';"
        "      isStreaming = wasStreaming;"
        "    }"
        "  }catch(e){"
        "    statusDiv.innerHTML='<span class=\"inactive\">Error: '+e.message+'</span>';"
        "    isStreaming = wasStreaming;"
        "  }"
        "  captureBtn.disabled=false;"
        "}"
        "</script>"
        "</body></html>";
    
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, strlen(html));
}

esp_err_t handleStartStreamRequest(httpd_req_t* req) {
    ESP_LOGI(TAG, "🟢 START requested");
    startVideoStream();
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    const char* body = "{\"status\":\"ok\",\"streaming\":true}";
    return httpd_resp_send(req, body, strlen(body));
}

esp_err_t handleStopStreamRequest(httpd_req_t* req) {
    ESP_LOGI(TAG, "🔴 STOP requested");
    stopVideoStream();
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    const char* body = "{\"status\":\"ok\",\"streaming\":false}";
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

// Handler для захоплення одного кадру (фотографія)
esp_err_t handleCaptureRequest(httpd_req_t* req) {
    camera_fb_t* fb = NULL;
    esp_err_t res = ESP_OK;
    
    ESP_LOGI(TAG, "📸 Capture photo requested");
    
    if (!camera_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Захоплюємо мьютекс перед доступом до камери
    if (xSemaphoreTake(camera_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire camera mutex for capture");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Захоплюємо кадр
    fb = esp_camera_fb_get();
    
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        xSemaphoreGive(camera_mutex);  // Звільняємо мьютекс
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    // Встановлюємо заголовки
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    // Додаємо timestamp
    char ts[32];
    snprintf(ts, sizeof(ts), "%lld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
    httpd_resp_set_hdr(req, "X-Timestamp", ts);
    
    // Відправляємо JPEG
    res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
    
    ESP_LOGI(TAG, "✅ Photo captured: %u bytes", fb->len);
    
    // Звільняємо буфер і мьютекс
    esp_camera_fb_return(fb);
    xSemaphoreGive(camera_mutex);
    
    return res;
}

// СТРІМ НА ОКРЕМОМУ СЕРВЕРІ (порт 81)
esp_err_t handleStreamRequest(httpd_req_t* req) {
    camera_fb_t* fb = NULL;
    esp_err_t res = ESP_OK;
    char part_buf[128];
    
    ESP_LOGI(TAG, "📹 Stream client connected");
    
    // Встановлюємо тип контенту
    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }
    
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "10");
    
    int frame_count = 0;
    
    while (streaming_active) {
        // Захоплюємо мьютекс перед доступом до камери
        if (xSemaphoreTake(camera_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            // Якщо не можемо отримати мьютекс (напр. фото робиться), пропускаємо кадр
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        fb = esp_camera_fb_get();
        
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            xSemaphoreGive(camera_mutex);  // Звільняємо мьютекс
            res = ESP_FAIL;
            break;
        }
        
        // Відправляємо boundary
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res != ESP_OK) {
            esp_camera_fb_return(fb);
            xSemaphoreGive(camera_mutex);
            break;
        }
        
        // Відправляємо заголовок кадру
        size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
        res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res != ESP_OK) {
            esp_camera_fb_return(fb);
            xSemaphoreGive(camera_mutex);
            break;
        }
        
        // Відправляємо JPEG дані
        res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
        if (res != ESP_OK) {
            esp_camera_fb_return(fb);
            xSemaphoreGive(camera_mutex);
            break;
        }
        
        esp_camera_fb_return(fb);
        fb = NULL;
        
        // Звільняємо мьютекс ОДРАЗУ після отримання кадру
        xSemaphoreGive(camera_mutex);
        
        frame_count++;
        if (frame_count % 50 == 0) {
            ESP_LOGI(TAG, "Streamed %d frames", frame_count);
        }
        
        // Затримка для ~10 FPS
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Завершуємо chunked response
    httpd_resp_send_chunk(req, NULL, 0);
    
    ESP_LOGI(TAG, "🔴 Stream ended after %d frames", frame_count);
    return res;
}


// Ініціалізація ДВОХ серверів


bool initWebServer(uint16_t port) {
    if (control_server != NULL || stream_server != NULL) {
        ESP_LOGW(TAG, "Web servers already running");
        return true;
    }

    // СЕРВЕР 1: Контроль (порт 80)
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = port;
    config.ctrl_port = 32768;
    config.max_open_sockets = 5;
    config.max_uri_handlers = 8;
    config.stack_size = 4096;
    config.task_priority = 5;
    config.core_id = 0;  // Ядро 0 для контролю
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting control server on port %d...", config.server_port);
    
    if (httpd_start(&control_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start control server");
        return false;
    }

    // Реєструємо контрольні handlers
    httpd_uri_t uri_root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = handleRootRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(control_server, &uri_root);

    httpd_uri_t uri_start = {
        .uri = "/stream/start",
        .method = HTTP_GET,
        .handler = handleStartStreamRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(control_server, &uri_start);

    httpd_uri_t uri_stop = {
        .uri = "/stream/stop",
        .method = HTTP_GET,
        .handler = handleStopStreamRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(control_server, &uri_stop);

    httpd_uri_t uri_status = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = handleStatusRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(control_server, &uri_status);

    httpd_uri_t uri_capture = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = handleCaptureRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(control_server, &uri_capture);

    ESP_LOGI(TAG, "✅ Control server started on port %d", port);

    // СЕРВЕР 2: Стрім (порт 81)
    config.server_port = port + 1;
    config.ctrl_port = 32769;
    config.max_open_sockets = 2;  // Тільки для стріму
    config.max_uri_handlers = 2;
    config.stack_size = 8192;     // Більше для стріму
    config.core_id = 1;           // Ядро 1 для стріму

    ESP_LOGI(TAG, "Starting stream server on port %d...", config.server_port);
    
    if (httpd_start(&stream_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start stream server");
        httpd_stop(control_server);
        control_server = NULL;
        return false;
    }

    // Реєструємо тільки стрім handler
    httpd_uri_t uri_stream = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = handleStreamRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(stream_server, &uri_stream);

    ESP_LOGI(TAG, "✅ Stream server started on port %d", port + 1);
    ESP_LOGI(TAG, "🎯 Architecture: Control (port %d, core 0) | Stream (port %d, core 1)", 
             port, port + 1);
    
    return true;
}

void stopWebServer() {
    if (stream_server != NULL) {
        httpd_stop(stream_server);
        stream_server = NULL;
        ESP_LOGI(TAG, "Stream server stopped");
    }
    
    if (control_server != NULL) {
        httpd_stop(control_server);
        control_server = NULL;
        ESP_LOGI(TAG, "Control server stopped");
    }
    
    if (camera_mutex != NULL) {
        vSemaphoreDelete(camera_mutex);
        camera_mutex = NULL;
        ESP_LOGI(TAG, "Camera mutex deleted");
    }
}

httpd_handle_t getServerHandle() {
    return control_server;
}