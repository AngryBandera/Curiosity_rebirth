#include "camera_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

static const char* TAG = "CAMERA_SERVER";

// Глобальні змінні
static httpd_handle_t server = NULL;
static bool camera_initialized = false;
static volatile bool streaming_active = false;
static SemaphoreHandle_t stream_mutex = NULL;
static int active_stream_clients = 0;
static SemaphoreHandle_t clients_mutex = NULL;

// ⭐ НАЛАШТУВАННЯ СТРІМУ - КОРОТШІ ПАКЕТИ!
#define FRAMES_PER_BATCH 3         // Тільки 3 кадри (~300мс)
#define BATCH_PAUSE_MS 500         // ДОВША пауза 500мс для кнопок
#define FRAME_DELAY_MS 100         // 100мс між кадрами (~10 FPS)

// WebSocket для контролю
static int control_ws_fd = -1;

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

    // Створюємо мьютекси для синхронізації
    stream_mutex = xSemaphoreCreateMutex();
    if (stream_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create stream mutex");
        return false;
    }

    clients_mutex = xSemaphoreCreateMutex();
    if (clients_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create clients mutex");
        vSemaphoreDelete(stream_mutex);
        stream_mutex = NULL;
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
    
    if (xSemaphoreTake(stream_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        streaming_active = true;
        xSemaphoreGive(stream_mutex);
        ESP_LOGI(TAG, "🎥 Video streaming STARTED");
        return true;
    }
    return false;
}

bool stopVideoStream() {
    ESP_LOGI(TAG, "🛑 Attempting to STOP streaming...");
    
    if (xSemaphoreTake(stream_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        streaming_active = false;
        xSemaphoreGive(stream_mutex);
        ESP_LOGI(TAG, "✅ Video streaming STOPPED");
        vTaskDelay(pdMS_TO_TICKS(300));
        return true;
    }
    return false;
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
// ⭐ WebSocket для контролю (окреме з'єднання!)
// ============================================

esp_err_t handleWebSocketControl(httpd_req_t* req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "🔌 WebSocket handshake");
        control_ws_fd = httpd_req_to_sockfd(req);
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed");
        return ret;
    }

    if (ws_pkt.len) {
        uint8_t* buf = (uint8_t*)calloc(1, ws_pkt.len + 1);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            free(buf);
            return ret;
        }

        ESP_LOGI(TAG, "📨 WebSocket received: %s", ws_pkt.payload);

        const char* response = NULL;
        
        // Обробка команд
        if (strcmp((char*)ws_pkt.payload, "START") == 0) {
            ESP_LOGI(TAG, "🟢 WS: START command");
            if (startVideoStream()) {
                response = "{\"status\":\"ok\",\"streaming\":true,\"via\":\"websocket\"}";
            } else {
                response = "{\"status\":\"error\",\"streaming\":false}";
            }
        } 
        else if (strcmp((char*)ws_pkt.payload, "STOP") == 0) {
            ESP_LOGI(TAG, "🔴 WS: STOP command");
            if (stopVideoStream()) {
                response = "{\"status\":\"ok\",\"streaming\":false,\"via\":\"websocket\"}";
            } else {
                response = "{\"status\":\"error\",\"streaming\":true}";
            }
        }
        else if (strcmp((char*)ws_pkt.payload, "STATUS") == 0) {
            char status_buf[128];
            snprintf(status_buf, sizeof(status_buf),
                     "{\"streaming\":%s,\"camera\":\"%s\",\"clients\":%d}",
                     streaming_active ? "true" : "false",
                     camera_initialized ? "ready" : "not_ready",
                     active_stream_clients);
            response = status_buf;
        }

        if (response) {
            ws_pkt.payload = (uint8_t*)response;
            ws_pkt.len = strlen(response);
            ws_pkt.type = HTTPD_WS_TYPE_TEXT;
            httpd_ws_send_frame(req, &ws_pkt);
        }

        free(buf);
    }

    return ESP_OK;
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
        "#debug{font-size:12px;color:#888;margin-top:10px}"
        ".ws-status{color:#4CAF50;font-size:12px}"
        "</style></head>"
        "<body>"
        "<h1>🚀 Mars Rover Camera</h1>"
        "<div id='streamContainer'>Press START to begin streaming</div>"
        "<div class='controls'>"
        "<button id='startBtn' onclick='startStream()'>▶️ START</button>"
        "<button id='stopBtn' class='stop' onclick='stopStream()' disabled>⏹ STOP</button>"
        "</div>"
        "<div id='status'>Ready</div>"
        "<div id='debug'>"
        "<div class='ws-status' id='wsStatus'>WebSocket: connecting...</div>"
        "Stream: 3 frames (300ms) → 500ms pause → repeat"
        "</div>"
        "<script>"
        "let container=document.getElementById('streamContainer');"
        "let statusDiv=document.getElementById('status');"
        "let debugDiv=document.getElementById('debug');"
        "let wsStatusDiv=document.getElementById('wsStatus');"
        "let startBtn=document.getElementById('startBtn');"
        "let stopBtn=document.getElementById('stopBtn');"
        "let streamImg=null;"
        "let isStreaming=false;"
        "let ws=null;"
        
        // ⭐ WebSocket для контролю (окреме з'єднання!)
        "function connectWebSocket(){"
        "  let wsUrl='ws://'+window.location.hostname+':'+window.location.port+'/ws';"
        "  console.log('Connecting to WebSocket:',wsUrl);"
        "  ws=new WebSocket(wsUrl);"
        "  ws.onopen=()=>{"
        "    console.log('✅ WebSocket connected');"
        "    wsStatusDiv.textContent='WebSocket: ✅ connected';"
        "    wsStatusDiv.style.color='#4CAF50';"
        "  };"
        "  ws.onclose=()=>{"
        "    console.log('❌ WebSocket disconnected');"
        "    wsStatusDiv.textContent='WebSocket: ❌ disconnected - reconnecting...';"
        "    wsStatusDiv.style.color='#f44336';"
        "    setTimeout(connectWebSocket,2000);"
        "  };"
        "  ws.onerror=(e)=>{"
        "    console.error('WebSocket error:',e);"
        "    wsStatusDiv.textContent='WebSocket: ⚠️ error';"
        "    wsStatusDiv.style.color='#ff9800';"
        "  };"
        "  ws.onmessage=(e)=>{"
        "    console.log('📨 WS response:',e.data);"
        "    try{"
        "      let data=JSON.parse(e.data);"
        "      if(data.streaming!==undefined){"
        "        if(data.streaming && !isStreaming){"
        "          console.log('Stream started via WS');"
        "          loadStream();"
        "        }else if(!data.streaming && isStreaming){"
        "          console.log('Stream stopped via WS');"
        "          unloadStream();"
        "        }"
        "      }"
        "    }catch(err){console.error('Parse error:',err);}"
        "  };"
        "}"
        
        "function sendWS(cmd){"
        "  if(ws && ws.readyState===WebSocket.OPEN){"
        "    console.log('📤 Sending WS:',cmd);"
        "    ws.send(cmd);"
        "    return true;"
        "  }else{"
        "    console.error('WebSocket not ready');"
        "    wsStatusDiv.textContent='WebSocket: ⚠️ not connected';"
        "    return false;"
        "  }"
        "}"
        
        "function loadStream(){"
        "  isStreaming=true;"
        "  container.innerHTML='';"
        "  streamImg=document.createElement('img');"
        "  streamImg.style.width='100%';"
        "  streamImg.style.height='auto';"
        "  streamImg.onerror=()=>{"
        "    console.error('Stream error');"
        "    debugDiv.textContent='Stream error';"
        "  };"
        "  container.appendChild(streamImg);"
        "  streamImg.src='/stream?_t='+Date.now();"
        "  statusDiv.innerHTML='<span class=\"active\">🎥 STREAMING (via WebSocket)</span>';"
        "  stopBtn.disabled=false;"
        "  startBtn.disabled=true;"
        "}"
        
        "function unloadStream(){"
        "  isStreaming=false;"
        "  if(streamImg){"
        "    streamImg.src='';"
        "    streamImg.remove();"
        "    streamImg=null;"
        "  }"
        "  container.innerHTML='Stream stopped - Press START to resume';"
        "  statusDiv.innerHTML='<span class=\"inactive\">⏸ STOPPED</span>';"
        "  startBtn.disabled=false;"
        "  stopBtn.disabled=true;"
        "}"
        
        "function startStream(){"
        "  if(isStreaming)return;"
        "  startBtn.disabled=true;"
        "  if(sendWS('START')){"
        "    console.log('START command sent via WebSocket');"
        "  }else{"
        "    startBtn.disabled=false;"
        "  }"
        "}"
        
        "function stopStream(){"
        "  if(!isStreaming)return;"
        "  stopBtn.disabled=true;"
        "  if(sendWS('STOP')){"
        "    console.log('STOP command sent via WebSocket');"
        "  }else{"
        "    stopBtn.disabled=false;"
        "  }"
        "}"
        
        // Підключаємося до WebSocket при завантаженні
        "connectWebSocket();"
        
        "</script>"
        "</body></html>";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, html, strlen(html));
}

// ⭐ КОРОТКІ ПАКЕТИ: 3 кадри → пауза 500мс
esp_err_t handleStreamRequest(httpd_req_t* req) {
    ESP_LOGI(TAG, "📹 Stream client connected on core %d", xPortGetCoreID());
    
    if (!camera_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Безпечно збільшуємо лічильник
    if (xSemaphoreTake(clients_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        active_stream_clients++;
        ESP_LOGI(TAG, "Active stream clients: %d", active_stream_clients);
        xSemaphoreGive(clients_mutex);
    }

    camera_fb_t* fb = NULL;
    esp_err_t res = ESP_OK;
    char part_buf[128];
    
    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Expires", "0");
    
    ESP_LOGI(TAG, "🟢 Stream loop: %d frames per batch, %dms pause", 
             FRAMES_PER_BATCH, BATCH_PAUSE_MS);
    
    int total_frames = 0;
    int batch_count = 0;
    
    while (streaming_active) {
        // ⭐ ПАКЕТ: відправляємо тільки 3 кадри (~300мс)
        for (int frame_in_batch = 0; frame_in_batch < FRAMES_PER_BATCH; frame_in_batch++) {
            if (!streaming_active) {
                ESP_LOGI(TAG, "Stream stopped during batch");
                break;
            }
            
            fb = esp_camera_fb_get();
            if (!fb) {
                ESP_LOGW(TAG, "Frame capture failed");
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            size_t hlen = snprintf(part_buf, sizeof(part_buf),
                "--frame\r\n"
                "Content-Type: image/jpeg\r\n"
                "Content-Length: %u\r\n"
                "\r\n",
                fb->len);

            res = httpd_resp_send_chunk(req, part_buf, hlen);
            if (res != ESP_OK) {
                esp_camera_fb_return(fb);
                goto stream_end;
            }

            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
            if (res != ESP_OK) {
                esp_camera_fb_return(fb);
                goto stream_end;
            }

            res = httpd_resp_send_chunk(req, "\r\n", 2);
            if (res != ESP_OK) {
                esp_camera_fb_return(fb);
                goto stream_end;
            }

            esp_camera_fb_return(fb);
            fb = NULL;
            total_frames++;
            
            vTaskDelay(pdMS_TO_TICKS(FRAME_DELAY_MS));
        }
        
        batch_count++;
        
        if (!streaming_active) {
            break;
        }
        
        // ⭐ ДОВГА ПАУЗА 500мс - WebSocket встигає обробити команди!
        if (batch_count % 10 == 0) {
            ESP_LOGI(TAG, "📦 Batch %d complete (%d total frames), PAUSING %dms", 
                     batch_count, total_frames, BATCH_PAUSE_MS);
        }
        vTaskDelay(pdMS_TO_TICKS(BATCH_PAUSE_MS));
    }

stream_end:
    if (res == ESP_OK) {
        httpd_resp_send_chunk(req, NULL, 0);
    }

    // Безпечно зменшуємо лічильник
    if (xSemaphoreTake(clients_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        active_stream_clients--;
        ESP_LOGI(TAG, "🔴 Stream ended. Total: %d frames, %d batches, Active clients: %d", 
                 total_frames, batch_count, active_stream_clients);
        xSemaphoreGive(clients_mutex);
    }
    
    return res;
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
    config.stack_size = 10240;
    config.task_priority = 5;
    config.core_id = tskNO_AFFINITY;
    
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 3;
    config.send_wait_timeout = 3;
    config.backlog_conn = 5;

    ESP_LOGI(TAG, "Starting web server...");
    
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start web server");
        return false;
    }

    // HTML сторінка
    httpd_uri_t uri_root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = handleRootRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &uri_root);

    // ⭐ WebSocket для контролю (окреме з'єднання)
    httpd_uri_t uri_ws = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = handleWebSocketControl,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &uri_ws);

    // MJPEG стрім (окреме з'єднання)
    httpd_uri_t uri_stream = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = handleStreamRequest,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &uri_stream);

    ESP_LOGI(TAG, "✅ Web server started on port %d", port);
    ESP_LOGI(TAG, "📊 Config: %d frames/batch (%dms), %dms pause between batches", 
             FRAMES_PER_BATCH, FRAMES_PER_BATCH * FRAME_DELAY_MS, BATCH_PAUSE_MS);
    ESP_LOGI(TAG, "🔌 WebSocket control available at /ws");
    return true;
}

void stopWebServer() {
    if (server != NULL) {
        stopVideoStream();
        vTaskDelay(pdMS_TO_TICKS(500));
        httpd_stop(server);
        server = NULL;
        ESP_LOGI(TAG, "Web server stopped");
    }
    
    if (stream_mutex != NULL) {
        vSemaphoreDelete(stream_mutex);
        stream_mutex = NULL;
    }
    
    if (clients_mutex != NULL) {
        vSemaphoreDelete(clients_mutex);
        clients_mutex = NULL;
    }
}

httpd_handle_t getServerHandle() {
    return server;
}