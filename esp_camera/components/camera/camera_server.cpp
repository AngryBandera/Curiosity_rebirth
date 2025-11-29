#include "camera_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string>
#include "esp_camera.h"
#include <string.h>
#include <stdio.h>
#include "esp_http_server.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

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
// New: Small helper SVG frames (on/off)
// ============================================
static const char* svg_stream_on_template =
    "<svg xmlns='http://www.w3.org/2000/svg' width='640' height='480'>"
    "<rect width='100%%' height='100%%' fill='#111'/>"
    "<text x='50%%' y='24%%' font-family='Arial' font-size='20' fill='#0f0' text-anchor='middle'>"
    "STREAM ON</text>"
    "<text x='50%%' y='62%%' font-family='Arial' font-size='14' fill='#fff' text-anchor='middle'>"
    "%s</text></svg>";

static const char* svg_stream_off =
    "<svg xmlns='http://www.w3.org/2000/svg' width='640' height='480'>"
    "<rect width='100%%' height='100%%' fill='#111'/>"
    "<text x='50%%' y='50%%' font-family='Arial' font-size='28' fill='#f00' text-anchor='middle'>"
    "STREAM OFF</text></svg>";

// ============================================
// Окремі таски і socket-based stream ВИДАЛЕНО
// Streaming now handled in HTTP handler (per-client)
// ============================================

// ============================================
// Функції для камери (init may fail but streaming still works)
// ============================================
bool initCamera(const camera_config_params_t* config) {
    if (camera_initialized) {
        ESP_LOGW(TAG, "Camera already initialized");
        return true;
    }

    capture_mutex = xSemaphoreCreateMutex();
    if (!capture_mutex) {
        ESP_LOGE(TAG, "Failed to create capture mutex");
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
        ESP_LOGW(TAG, "Camera init failed: 0x%x (continuing without camera)", err);
        camera_initialized = false; // still allow streaming without camera
        return true; // return true so user can run webserver without camera
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
    ESP_LOGI(TAG, "✅ Camera initialized (hardware present)");
    return true;
}

photo_data_t capturePhoto() {
    photo_data_t photo = {NULL, 0};
    if (!camera_initialized) {
        return photo;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
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
    if (streaming_active) return "Streaming";
    return "Ready";
}

void requestCaptureFromStream() {
    capture_request_flag = true;
    capture_ready = false;
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
        "img{max-width:100%;height:auto;border:2px solid #4CAF50;border-radius:8px;background:#000}"
        "button{padding:12px 24px;margin:10px;font-size:16px;background:#4CAF50;color:#fff;"
        "border:none;border-radius:5px;cursor:pointer;transition:0.3s}"
        "button:hover{background:#45a049}"
        "button:disabled{background:#666;cursor:not-allowed}"
        "#status{padding:10px;background:#333;border-radius:5px;margin:10px auto;max-width:400px}"
        ".success{color:#4CAF50}.error{color:#f44336}.warning{color:#ff9800}"
        "</style></head>"
        "<body><h1>🚀 Mars Rover Camera</h1>"
        "<img id='stream' src='/stream' alt='Loading stream...' onerror=\"this.alt='Stream error'\">"
        "<br>"
        "<button id='startBtn' onclick='startStream()'>▶️ Start Stream</button>"
        "<button id='stopBtn' onclick='stopStream()'>⏹ Stop Stream</button>"
        "<br>"
        "<button id='captureBtn' onclick='capturePhoto()'>📷 Capture (from camera)</button>"
        "<button id='quickBtn' onclick='quickCapture()'>⚡ Quick Capture</button>"
        "<button onclick='location.reload()'>🔄 Refresh</button>"
        "<p id='status'>Ready • Streaming active</p>"
        "<script>"
        "let capturing=false;"
        "async function startStream(){"
        " try{ await fetch('/stream/start');"
        " document.getElementById('status').textContent='Ready • Streaming active';"
        " // reload the image stream to reflect new state"
        " document.getElementById('stream').src='/stream?rnd='+Date.now(); }catch(e){} }"
        "async function stopStream(){"
        " try{ await fetch('/stream/stop');"
        " document.getElementById('status').textContent='Ready • Streaming stopped';"
        " document.getElementById('stream').src='/stream?rnd='+Date.now(); }catch(e){} }"
        "async function capturePhoto(){"
        " if(capturing)return;"
        " capturing=true;"
        " const btn=document.getElementById('captureBtn');"
        " const st=document.getElementById('status');"
        " btn.disabled=true;"
        " st.innerHTML='<span class=\"warning\">📸 Capturing...</span>';"
        " try{ const r=await fetch('/capture');"
        " if(!r.ok)throw new Error('Server error');"
        " const b=await r.blob();"
        " const u=URL.createObjectURL(b);"
        " const a=document.createElement('a'); a.href=u; a.download='capture_'+Date.now()+'.jpg';"
        " document.body.appendChild(a); a.click(); document.body.removeChild(a);"
        " setTimeout(()=>URL.revokeObjectURL(u),100);"
        " st.innerHTML='<span class=\"success\">✅ Photo saved!</span>';"
        " setTimeout(()=>{st.innerHTML='Ready • Streaming active'},3000);"
        " }catch(e){ st.innerHTML='<span class=\"error\">❌ '+e.message+'</span>'; setTimeout(()=>{st.innerHTML='Ready • Streaming active'},3000); }"
        " finally{ capturing=false; btn.disabled=false; } }"
        "async function quickCapture(){"
        " if(capturing)return;"
        " capturing=true;"
        " const btn=document.getElementById('quickBtn');"
        " const st=document.getElementById('status');"
        " btn.disabled=true; st.innerHTML='<span class=\"warning\">⚡ Quick capture...</span>';"
        " try{ const r=await fetch('/quick'); if(!r.ok)throw new Error('Server error');"
        " const b=await r.blob(); const u=URL.createObjectURL(b); const a=document.createElement('a'); a.href=u; a.download='quick_'+Date.now()+'.jpg';"
        " document.body.appendChild(a); a.click(); document.body.removeChild(a); setTimeout(()=>URL.revokeObjectURL(u),100);"
        " st.innerHTML='<span class=\"success\">⚡ Quick photo!</span>'; setTimeout(()=>{st.innerHTML='Ready • Streaming active'},3000); }"
        " catch(e){ st.innerHTML='<span class=\"error\">❌ '+e.message+'</span>'; setTimeout(()=>{st.innerHTML='Ready • Streaming active'},3000); }"
        " finally{ capturing=false; btn.disabled=false; } }"
        "setInterval(async()=>{try{const r=await fetch('/status'); const d=await r.json(); const st=document.getElementById('status');"
        " st.textContent = d.streaming ? 'Ready • Streaming active' : 'Ready • Streaming stopped'; }catch(e){} },3000);"
        "</script></body></html>";
    
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, strlen(html));
}

// ...existing code...
esp_err_t handleStreamRequest(httpd_req_t* req) {
    ESP_LOGI(TAG, "🎬 Stream request (no camera required)");

    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

    char frame_header[128];
    char svg_buf[512];
    char status_line[64]; // <--- added

    while (true) {
        bool active = streaming_active;
        const char* payload;
        int payload_len = 0;

        if (active) {
            // форматування timestamp без std::to_string
            uint64_t ms = esp_timer_get_time() / 1000ULL; // ms
            snprintf(status_line, sizeof(status_line), "Live • %llu ms", (unsigned long long)ms);
            snprintf(svg_buf, sizeof(svg_buf), svg_stream_on_template, status_line);
            payload = svg_buf;
            payload_len = (int)strlen(svg_buf);
        } else {
            payload = svg_stream_off;
            payload_len = (int)strlen(svg_stream_off);
        }

        int header_len = snprintf(frame_header, sizeof(frame_header),
                                  "--frame\r\nContent-Type: image/svg+xml\r\nContent-Length: %d\r\n\r\n",
                                  payload_len);

        // send header + payload + terminator
        if (httpd_resp_send_chunk(req, frame_header, header_len) != ESP_OK) {
            ESP_LOGI(TAG, "Stream client disconnected (header send failed)");
            break;
        }
        if (httpd_resp_send_chunk(req, payload, payload_len) != ESP_OK) {
            ESP_LOGI(TAG, "Stream client disconnected (payload send failed)");
            break;
        }
        if (httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) {
            ESP_LOGI(TAG, "Stream client disconnected (terminator send failed)");
            break;
        }

        // If streaming is active send at higher rate, else slow
        if (streaming_active) {
            vTaskDelay(pdMS_TO_TICKS(200)); // 5 FPS
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000)); // show off-image once per second while idle
        }
    }

    // finalize multipart
    httpd_resp_send_chunk(req, NULL, 0);
    ESP_LOGI(TAG, "📴 Stream handler exiting");
    return ESP_OK;
}
// ...existing code...
esp_err_t handleStartStreamRequest(httpd_req_t* req) {
    ESP_LOGI(TAG, "🟢 Start stream requested");
    startVideoStream();
    httpd_resp_set_type(req, "application/json");
    const char* body = "{\"status\":\"started\"}";
    httpd_resp_send(req, body, strlen(body));
    return ESP_OK;
}

esp_err_t handleStopStreamRequest(httpd_req_t* req) {
    ESP_LOGI(TAG, "🔴 Stop stream requested");
    stopVideoStream();
    httpd_resp_set_type(req, "application/json");
    const char* body = "{\"status\":\"stopped\"}";
    httpd_resp_send(req, body, strlen(body));
    return ESP_OK;
}

esp_err_t handleCaptureRequest(httpd_req_t *req) {
    ESP_LOGI(TAG, "📷 Capture request received");
    
    if (!camera_initialized) {
        // If no camera, return an informative SVG image
        const char* no_camera_svg =
            "<svg xmlns='http://www.w3.org/2000/svg' width='640' height='480'>"
            "<rect width='100%%' height='100%%' fill='#111'/>"
            "<text x='50%%' y='50%%' font-family='Arial' font-size='28' fill='#f00' text-anchor='middle'>"
            "No Camera Available</text></svg>";
        httpd_resp_set_type(req, "image/svg+xml");
        httpd_resp_send(req, no_camera_svg, strlen(no_camera_svg));
        return ESP_OK;
    }

    // Встановлюємо флаг -> allow stream_task to capture if active
    requestCaptureFromStream();

    // Чекаємо готовність (максимум 3 секунди)
    int timeout_ms = 3000;
    int waited_ms = 0;
    
    while (!isCaptureReady() && waited_ms < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(10));
        waited_ms += 10;
    }
    
    if (!isCaptureReady()) {
        ESP_LOGW(TAG, "Capture from stream timed out after %dms - trying direct capture", waited_ms);
        // fallback: direct capture using camera if present
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) {
            if (fb->len > 2 && fb->buf[0] == 0xFF && fb->buf[1] == 0xD8) {
                httpd_resp_set_type(req, "image/jpeg");
                httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
                esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
                esp_camera_fb_return(fb);
                return res;
            }
            esp_camera_fb_return(fb);
        }
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Timeout");
        return ESP_FAIL;
    }
    
    // Отримуємо збережене фото
    photo_data_t photo = getCapturedPhoto();
    
    if (!photo.buffer || photo.length == 0) {
        ESP_LOGE(TAG, "❌ No photo data");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No data");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "📤 Sending captured photo: %d bytes", photo.length);
    
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    esp_err_t res = httpd_resp_send(req, (const char*)photo.buffer, photo.length);
    
    free(photo.buffer);
    
    if (res == ESP_OK) {
        ESP_LOGI(TAG, "✅ Photo sent successfully");
    } else {
        ESP_LOGE(TAG, "❌ Failed to send photo");
    }
    
    return res;
}

esp_err_t handleStatusRequest(httpd_req_t* req) {
    char json[256];
    snprintf(json, sizeof(json),
             "{\"camera\":\"%s\",\"streaming\":%s,\"fps\":5}",
             camera_initialized ? "ready" : "not_ready",
             streaming_active ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, strlen(json));
}

// Новий швидкий endpoint - захоплює кадр напряму без стріму
esp_err_t handleQuickCaptureRequest(httpd_req_t* req) {
    ESP_LOGI(TAG, "⚡ Quick capture request");
    
    if (!camera_initialized) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Camera not ready");
        return ESP_FAIL;
    }
    
    // Пробуємо до 3 разів отримати валідний кадр
    for (int attempt = 0; attempt < 3; attempt++) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Attempt %d: Failed to get frame", attempt + 1);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        
        // Перевіряємо валідність JPEG
        if (fb->len > 2 && fb->buf[0] == 0xFF && fb->buf[1] == 0xD8) {
            ESP_LOGI(TAG, "✅ Quick capture: %d bytes (attempt %d)", fb->len, attempt + 1);
            
            httpd_resp_set_type(req, "image/jpeg");
            httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=quick_capture.jpg");
            httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
            esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
            
            esp_camera_fb_return(fb);
            return res;
        } else {
            ESP_LOGW(TAG, "Attempt %d: Invalid JPEG", attempt + 1);
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    
    ESP_LOGE(TAG, "❌ Failed to get valid frame after 3 attempts");
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to capture");
    return ESP_FAIL;
}

bool initWebServer(uint16_t port) {
    if (server != NULL) {
        return true;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = port;
    config.max_open_sockets = 7;
    config.max_uri_handlers = 12;  // Збільшено для нового endpoint
    config.task_priority = 5;
    config.stack_size = 4096;
    config.lru_purge_enable = true;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start server");
        return false;
    }

    httpd_uri_t uri_root = {"/", HTTP_GET, handleRootRequest, NULL};
    httpd_uri_t uri_stream = {"/stream", HTTP_GET, handleStreamRequest, NULL};
    httpd_uri_t uri_stream_start = {"/stream/start", HTTP_GET, handleStartStreamRequest, NULL};
    httpd_uri_t uri_stream_stop = {"/stream/stop", HTTP_GET, handleStopStreamRequest, NULL};
    httpd_uri_t uri_capture = {"/capture", HTTP_GET, handleCaptureRequest, NULL};
    httpd_uri_t uri_quick = {"/quick", HTTP_GET, handleQuickCaptureRequest, NULL};
    httpd_uri_t uri_status = {"/status", HTTP_GET, handleStatusRequest, NULL};

    httpd_register_uri_handler(server, &uri_root);
    httpd_register_uri_handler(server, &uri_stream);
    httpd_register_uri_handler(server, &uri_stream_start);
    httpd_register_uri_handler(server, &uri_stream_stop);
    httpd_register_uri_handler(server, &uri_capture);
    httpd_register_uri_handler(server, &uri_quick);
    httpd_register_uri_handler(server, &uri_status);

    ESP_LOGI(TAG, "✅ Server started on port %d", port);
    ESP_LOGI(TAG, "   Endpoints: /, /stream, /stream/start, /stream/stop, /capture, /quick, /status");
    return true;
}

void stopWebServer() {
    if (server != NULL) {
        httpd_stop(server);
        server = NULL;
    }
}

httpd_handle_t getServerHandle() {
    return server;
}