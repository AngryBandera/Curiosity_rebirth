//
// Created by bogda on 12/11/2025.
//

#ifndef CURIOSITY_REBIRTH_CAMERA_SERVER_H
#define CURIOSITY_REBIRTH_CAMERA_SERVER_H

#include "esp_camera.h"
#include "esp_http_server.h"
#include <stdint.h>
#include <stdbool.h>

// Структура для конфігурації камери
typedef struct {
    int pin_pwdn;
    int pin_reset;
    int pin_xclk;
    int pin_sscb_sda;
    int pin_sscb_scl;
    int pin_d7;
    int pin_d6;
    int pin_d5;
    int pin_d4;
    int pin_d3;
    int pin_d2;
    int pin_d1;
    int pin_d0;
    int pin_vsync;
    int pin_href;
    int pin_pclk;
    framesize_t frame_size;
    int jpeg_quality;
    int fb_count;
} camera_config_params_t;

// Структура для фото даних
typedef struct {
    uint8_t* buffer;
    size_t length;
} photo_data_t;

// Функції для камери
bool initCamera(const camera_config_params_t* config);
photo_data_t capturePhoto();
bool startVideoStream();
bool stopVideoStream();
bool isStreaming();
void releasePhotoBuffer(photo_data_t photo);

// Нова функція для capture прямо зі стріму
void requestCaptureFromStream();
bool isCaptureReady();
photo_data_t getCapturedPhoto();

// Функції для вебсервера
bool initWebServer(uint16_t port);
void stopWebServer();
httpd_handle_t getServerHandle();

// HTTP handler функції
esp_err_t handleStreamRequest(httpd_req_t* req);
esp_err_t handlePhotoRequest(httpd_req_t* req);
esp_err_t handleCaptureRequest(httpd_req_t* req);
esp_err_t handleQuickCaptureRequest(httpd_req_t* req);  // Новий швидкий endpoint
esp_err_t handleStatusRequest(httpd_req_t* req);
esp_err_t handleRootRequest(httpd_req_t* req);

// Допоміжні функції
const char* getCameraStatus();
bool isCameraInitialized();

#endif //CURIOSITY_REBIRTH_CAMERA_SERVER_H