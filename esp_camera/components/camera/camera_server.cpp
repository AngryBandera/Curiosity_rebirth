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
        "</style></head>"
        "<body>"
        "<h1>🚀 Mars Rover Camera</h1>"
        "<div id='streamContainer'>Press START to begin streaming</div>"
        "<div class='controls'>"
        "<button id='startBtn' onclick='startStream()'>▶️ START</button>"
        "<button id='stopBtn' class='stop' onclick='stopStream()' disabled>⏹ STOP</button>"
        "</div>"
        "<div id='status'>Ready</div>"
        "<div id='debug'>Stream: 3 frames (300ms) → 500ms pause → repeat</div>"
        "<script>"
        "let container=document.getElementById('streamContainer');"
        "let statusDiv=document.getElementById('status');"
        "let debugDiv=document.getElementById('debug');"
        "let startBtn=document.getElementById('startBtn');"
        "let stopBtn=document.getElementById('stopBtn');"
        "let streamImg=null;"
        "let isStreaming=false;"
        "let commandInProgress=false;"
        
        "async function sendCommand(cmd,endpoint){"
        "  if(commandInProgress){"
        "    console.log('Command already in progress, queuing...');"
        "    await new Promise(r=>setTimeout(r,100));"
        "  }"
        "  commandInProgress=true;"
        "  try{"
        "    console.log('Sending:',cmd,'to',endpoint);"
        "    let controller=new AbortController();"
        "    let timeout=setTimeout(()=>controller.abort(),2000);"
        "    let r=await fetch(endpoint,{"
        "      method:'GET',"
        "      cache:'no-cache',"
        "      signal:controller.signal"
        "    });"
        "    clearTimeout(timeout);"
        "    if(r.ok){"
        "      let data=await r.json();"
        "      console.log('Response:',data);"
        "      commandInProgress=false;"
        "      return data;"
        "    }"
        "  }catch(e){"
        "    console.error('Command error:',e);"
        "  }"
        "  commandInProgress=false;"
        "  return null;"
        "}"
        
        "async function startStream(){"
        "  if(isStreaming)return;"
        "  startBtn.disabled=true;"
        "  debugDiv.textContent='Sending START...';"
        "  let data=await sendCommand('START','/stream/start');"
        "  if(data && data.status=='ok'){"
        "    isStreaming=true;"
        "    container.innerHTML='';"
        "    streamImg=document.createElement('img');"
        "    streamImg.style.width='100%';"
        "    streamImg.style.height='auto';"
        "    streamImg.onerror=()=>{"
        "      console.error('Stream error');"
        "      debugDiv.textContent='Stream error';"
        "    };"
        "    container.appendChild(streamImg);"
        "    setTimeout(()=>{"
        "      streamImg.src='/stream?_t='+Date.now();"
        "    },200);"
        "    statusDiv.innerHTML='<span class=\"active\">🎥 STREAMING</span>';"
        "    debugDiv.textContent='Stream: 3 frames → 500ms pause (button check)';"
        "    stopBtn.disabled=false;"
        "  }else{"
        "    debugDiv.textContent='START failed';"
        "    startBtn.disabled=false;"
        "  }"
        "}"
        
        "async function stopStream(){"
        "  if(!isStreaming)return;"
        "  stopBtn.disabled=true;"
        "  debugDiv.textContent='Sending STOP...';"
        "  let data=await sendCommand('STOP','/stream/stop');"
        "  if(data && data.status=='ok'){"
        "    isStreaming=false;"
        "    if(streamImg){"
        "      streamImg.src='';"
        "      streamImg.remove();"
        "      streamImg=null;"
        "    }"
        "    container.innerHTML='Stream stopped - Press START to resume';"
        "    statusDiv.innerHTML='<span class=\"inactive\">⏸ STOPPED</span>';"
        "    debugDiv.textContent='Stream stopped';"
        "    startBtn.disabled=false;"
        "  }else{"
        "    debugDiv.textContent='STOP failed - wait for pause';"
        "    setTimeout(stopStream,600);"
        "  }"
        "}"
        
        "</script>"
        "</body></html>";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, html, strlen(html));
}

// ⭐ КОРОТКІ ПАКЕТИ: 3 кадри → пауза 500мс
// ⭐ КОРОТКІ ПАКЕТИ: 3 кадри → пауза 500мс
esp_err_t handleStreamRequest(httpd_req_t* req) {
esp_err_t handleStartStreamRequest(httpd_req_t* req) {
    ESP_LOGI(TAG, "🟢 START button pressed on core %d", xPortGetCoreID());
    
    if (!camera_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        const char* body = "{\"status\":\"error\",\"message\":\"camera not ready\"}";
        return httpd_resp_send(req, body, strlen(body));
    }
    
    bool started = startVideoStream();
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    
    char response[128];
    snprintf(response, sizeof(response), 
             "{\"status\":\"%s\",\"streaming\":%s,\"core\":%d}", 
             started ? "ok" : "error",
             started ? "true" : "false",
             xPortGetCoreID());
    
    ESP_LOGI(TAG, "✅ START response: %s", response);
    return httpd_resp_send(req, response, strlen(response));
}

esp_err_t handleStopStreamRequest(httpd_req_t* req) {
    ESP_LOGI(TAG, "🔴 STOP button pressed on core %d", xPortGetCoreID());
    
    bool stopped = stopVideoStream();
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    
    char response[128];
    snprintf(response, sizeof(response), 
             "{\"status\":\"%s\",\"streaming\":%s,\"core\":%d}", 
             stopped ? "ok" : "error",
             stopped ? "false" : "true",
             xPortGetCoreID());
    
    ESP_LOGI(TAG, "✅ STOP response: %s", response);
    return httpd_resp_send(req, response, strlen(response));
}

esp_err_t handleStatusRequest(httpd_req_t* req) {
    char json[128];
    
    int clients = 0;
    if (xSemaphoreTake(clients_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        clients = active_stream_clients;
        xSemaphoreGive(clients_mutex);
    }
    
    snprintf(json, sizeof(json),
             "{\"streaming\":%s,\"camera\":\"%s\",\"clients\":%d,\"core\":%d}",
             streaming_active ? "true" : "false",
             camera_initialized ? "ready" : "not_ready",
             clients,
             xPortGetCoreID());
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, json, strlen(json));
}
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

    // Контрольні ендпоінти (швидкі)
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

    // MJPEG стрім (може блокувати)
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