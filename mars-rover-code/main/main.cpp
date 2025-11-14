// C++ headers
#include <cstddef>
#include <cstdint>
#include <sys/types.h>
#include <string.h> 

// C headers
extern "C" {

    #include <string.h>
    #include <stdbool.h>
    #include <stdio.h>
    #include <inttypes.h>
    #include "nvs.h"
    #include "nvs_flash.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_log.h"
    #include "esp_bt.h"
    #include "esp_bt_main.h"
    #include "esp_gap_bt_api.h"
    #include "esp_bt_device.h"
    #include "esp_spp_api.h"
    #include "spp_task.h"
    #include "time.h"
    #include "sys/time.h"
    #include "esp_vfs.h"
    #include "sys/unistd.h"
    #include <ctype.h>
}

// Rover C++ headers
#include "motors.h"
#include "driver/ledc.h"
#include "pca9685.h"
// Bluetooth definitions
#define SPP_TAG "SPP_ROVER_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"

static const char local_device_name[] = CONFIG_EXAMPLE_LOCAL_DEVICE_NAME;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

#define SPP_DATA_LEN 100

// --- Global Rover Objects ---
// These are global so background tasks (BT) can control them.
static i2c_dev_t *g_pca9685_dev = nullptr;
static PCA9685Buffer* g_buffer = nullptr;
static DriveSystem *g_rover = nullptr;

// --- Constants for Rover Control ---
// You can adjust these values
#define ROVER_MAX_TURN_ANGLE 60.0f
#define ROVER_STOP_SPEED 0

static int base_speed = 1500;
static float curr_angle = 0.0f;
// --- C++/C Bridge Functions ---
// These are plain functions that call C++ methods on our global rover object.

void stopMotors() {
    if (g_rover) {
        g_rover->move(ROVER_STOP_SPEED, 0.0f);
        // ESP_LOGI(SPP_TAG, "Rover: Stop");
    }
}

// This wrapper is needed because all C-style callbacks must be in extern "C"
extern "C" {


    static char *bda2str(uint8_t * bda, char *str, size_t size)
    {
        if (bda == NULL || str == NULL || size < 18) {
            return NULL;
        }

        uint8_t *p = bda;
        sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
                p[0], p[1], p[2], p[3], p[4], p[5]);
        return str;
    }

    int hex_char_to_int(char c) {
        if (c >= '0' && c <= '9') {
            return c - '0';
        }
        if (c >= 'a' && c <= 'f') {
            return c - 'a' + 10;
        }
        if (c >= 'A' && c <= 'F') {
            return c - 'A' + 10;
        }
        return -1;
    }


    static void spp_read_handle(void * param)
    {
        int fd = (int)param;
        ssize_t size = 0; 
        uint8_t *spp_data = NULL;

        #define PROCESS_BUF_SIZE (SPP_DATA_LEN * 2)
        uint8_t *process_buffer = NULL;
        size_t process_buffer_len = 0;

        // --- Allocate buffers ---
        spp_data = (uint8_t *)malloc(SPP_DATA_LEN);
        if (!spp_data) {
            ESP_LOGE(SPP_TAG, "malloc spp_data failed, fd:%d", fd);
            goto done;
        }
        
        process_buffer = (uint8_t *)malloc(PROCESS_BUF_SIZE);
        if (!process_buffer) {
            ESP_LOGE(SPP_TAG, "malloc process_buffer failed, fd:%d", fd);
            goto done;
        }

        do {
            size = read(fd, spp_data, SPP_DATA_LEN);

            if (size < 0) {
                ESP_LOGE(SPP_TAG, "read() failed");
                break;
            } else if (size == 0) {
                vTaskDelay(500 / portTICK_PERIOD_MS);
            } else {
                if (process_buffer_len + size > PROCESS_BUF_SIZE) {
                    ESP_LOGE(SPP_TAG, "Process buffer overflow! Discarding all data.");
                    process_buffer_len = 0;
                }

                memcpy(&process_buffer[process_buffer_len], spp_data, size);
                process_buffer_len += size;

                ESP_LOGI(SPP_TAG, "Read %d bytes, total buffer %d", size, process_buffer_len);

                size_t parse_index = 0;

                while (parse_index < process_buffer_len) {
                    uint8_t current_cmd = process_buffer[parse_index];

                    if (current_cmd == 0x46 || current_cmd == 0x42) {
                       if (parse_index + 5 < process_buffer_len) {
                            int move_speed = 0;
                            int turn_degrees = 0;
                            int d1 = hex_char_to_int(process_buffer[parse_index + 1]);
                            int d2 = hex_char_to_int(process_buffer[parse_index + 2]);

                            if (d1 != -1 && d2 != -1) {
                                move_speed = ((d1 * 10) + d2);
                                parse_index += 3;
                            } else {
                                ESP_LOGW(SPP_TAG, "Invalid hex chars after 'F' or 'B' command, discarding 'F' or 'B'.");
                                parse_index += 1;
                                continue;
                            }

                            if (current_cmd == 0x42) {
                                move_speed = -move_speed;
                                ESP_LOGI(SPP_TAG, "CMD: BACKWARD, Speed: %d", move_speed);
                            } else {
                                ESP_LOGI(SPP_TAG, "CMD: FORWARD, Speed: %d", move_speed);
                            }
                            current_cmd = process_buffer[parse_index];
                            if (current_cmd != 0x4C && current_cmd != 0x52) {
                                ESP_LOGI(SPP_TAG, "Partial move command, waiting for more data.");
                                break;
                            }

                            d1 = hex_char_to_int(process_buffer[parse_index + 1]);
                            d2 = hex_char_to_int(process_buffer[parse_index + 2]);

                            if (d1 != -1 && d2 != -1) {
                                turn_degrees = ((d1 * 10) + d2);
                                parse_index += 3;
                            } else {
                                ESP_LOGW(SPP_TAG, "Invalid hex chars after 'L' or 'R', discarding 'L' or 'R'.");
                                parse_index += 1;
                                continue;
                            }

                            if (current_cmd == 0x4C) {
                                turn_degrees = -turn_degrees;
                                ESP_LOGI(SPP_TAG, "CMD: LEFT, Degrees: %d", turn_degrees);
                            } else {
                                ESP_LOGI(SPP_TAG, "CMD: RIGHT, Degrees: %d", turn_degrees);
                            }
                            g_rover->move(move_speed*20, static_cast<float>(turn_degrees));
                       } else {
                           break;
                       }
                    } else if (current_cmd == 0x00) {
                        ESP_LOGI(SPP_TAG, "CMD: STOP");
                        g_rover->move(0, 0.0f);
                        parse_index += 1;
                    }
                    else {
                        ESP_LOGW(SPP_TAG, "Unknown command byte: 0x%02X", current_cmd);
                        parse_index += 1;
                    }
                }

                if (parse_index == 0) {
                } else if (parse_index < process_buffer_len) {
                    size_t remaining_data = process_buffer_len - parse_index;
                    memmove(process_buffer, &process_buffer[parse_index], remaining_data);
                    process_buffer_len = remaining_data;
                    ESP_LOGD(SPP_TAG, "Shifted %d leftover bytes to front", remaining_data);
                } else {
                    process_buffer_len = 0;
                }
            }

            vTaskDelay(10 / portTICK_PERIOD_MS);
        } while (true);


    done:
        ESP_LOGE(SPP_TAG, "SPP read task is stopping...");
        if (spp_data) {
            free(spp_data);
        }
        if (process_buffer) {
            free(process_buffer);
        }
        vTaskDelete(NULL);
    }
    static void esp_spp_cb(uint16_t e, void *p)
    {
        // FIX: C++ requires explicit casts for enum and void*
        esp_spp_cb_event_t event = (esp_spp_cb_event_t)e;
        esp_spp_cb_param_t *param = (esp_spp_cb_param_t *)p;
        char bda_str[18] = {0};

        switch (event) {
        case ESP_SPP_INIT_EVT:
            if (param->init.status == ESP_SPP_SUCCESS) {
                ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
                /* Enable SPP VFS mode */
                esp_spp_vfs_register();
            } else {
                ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
            }
            break;
        case ESP_SPP_DISCOVERY_COMP_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
            break;
        case ESP_SPP_OPEN_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
            break;
        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%"PRIu32" close_by_remote:%d", param->close.status,
                    param->close.handle, param->close.async);
            // Safety stop on disconnect
            g_rover->move(0, 0.0f);
            break;
        case ESP_SPP_START_EVT:
            if (param->start.status == ESP_SPP_SUCCESS) {
                ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%"PRIu32" sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
                        param->start.scn);
                esp_bt_gap_set_device_name(local_device_name);
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            } else {
                ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
            }
            break;
        case ESP_SPP_CL_INIT_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%"PRIu32", rem_bda:[%s]", param->srv_open.status,
                    param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
            if (param->srv_open.status == ESP_SPP_SUCCESS) {
                spp_wr_task_start_up(spp_read_handle, param->srv_open.fd);
            }
            break;
        case ESP_SPP_VFS_REGISTER_EVT:
            if (param->vfs_register.status == ESP_SPP_SUCCESS) {
                ESP_LOGI(SPP_TAG, "ESP_SPP_VFS_REGISTER_EVT");
                esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
            } else {
                ESP_LOGE(SPP_TAG, "ESP_SPP_VFS_REGISTER_EVT status:%d", param->vfs_register.status);
            }
            break;
        default:
            break;
        }
    }

    static void esp_spp_stack_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
    {
        /* To avoid stucking Bluetooth stack, we dispatch the SPP callback event to the other lower priority task */
        spp_task_work_dispatch(esp_spp_cb, event, param, sizeof(esp_spp_cb_param_t), NULL);
    }

    void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
    {
        switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT:{
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
                ESP_LOG_BUFFER_HEX(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
            } else {
                ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
            }
            break;
        }
        case ESP_BT_GAP_PIN_REQ_EVT:{
            ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
            if (param->pin_req.min_16_digit) {
                ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
                esp_bt_pin_code_t pin_code = {0};
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
            } else {
                ESP_LOGI(SPP_TAG, "Input pin code: 1234");
                esp_bt_pin_code_t pin_code;
                pin_code[0] = '1';
                pin_code[1] = '2';
                pin_code[2] = '3';
                pin_code[3] = '4';
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            }
            break;
        }

    #if (CONFIG_EXAMPLE_SSP_ENABLED == true)
        case ESP_BT_GAP_CFM_REQ_EVT:
            ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %06" PRIu32, param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;
        case ESP_BT_GAP_KEY_NOTIF_EVT:
            // FIX: Add a space between the string literal and the C macro
            ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%06" PRIu32, param->key_notif.passkey);
            break;
        case ESP_BT_GAP_KEY_REQ_EVT:
            ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
            break;
    #endif

        case ESP_BT_GAP_MODE_CHG_EVT:
            ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
            break;

        default: {
            ESP_LOGI(SPP_TAG, "event: %d", event);
            break;
        }
        }
        return;
    }

    void app_main()
    {
        // --- 1. Rover Init ---
        // We use 'new' to create them on the heap, so they persist after app_main exits
        // and can be accessed by the global pointers.
        g_pca9685_dev = new i2c_dev_t;
        g_buffer = new PCA9685Buffer{g_pca9685_dev};
        

        // // Create the rover object and assign it to the global pointer
        g_rover = new DriveSystem(g_pca9685_dev);
        ESP_LOGI(SPP_TAG, "Rover DriveSystem Initialized.");
        g_rover->move(2000, 0.0f);

        // --- 2. Bluetooth Init ---
        char bda_str[18] = {0};
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK( ret );

        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s initialize controller failed", __func__);
            return;
        }

        if (esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s enable controller failed", __func__);
            return;
        }

        esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    #if (CONFIG_EXAMPLE_SSP_ENABLED == false)
        bluedroid_cfg.ssp_en = false;
    #endif
        if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
            return;
        }

        if (esp_bluedroid_enable() != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s enable bluedroid failed", __func__);
            return;
        }

        if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s gap register failed: %s", __func__, esp_err_to_name(ret));
            return;
        }

        if (esp_spp_register_callback(esp_spp_stack_cb) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s spp register failed", __func__);
            return;
        }

        spp_task_task_start_up();
        
        // start command runner task which repeatedly calls the current command
        // xTaskCreate(command_runner_task, "cmd_runner", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);

        esp_spp_cfg_t bt_spp_cfg = BT_SPP_DEFAULT_CONFIG();
        if (esp_spp_enhanced_init(&bt_spp_cfg) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s spp init failed", __func__);
            return;
        }

    #if (CONFIG_EXAMPLE_SSP_ENABLED == true)
        /* Set default parameters for Secure Simple Pairing */
        esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
        esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
        esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
    #endif

        /*
        * Set default parameters for Legacy Pairing
        * Use variable pin, input pin code when pairing
        */
        esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
        esp_bt_pin_code_t pin_code;
        esp_bt_gap_set_pin(pin_type, 0, pin_code);

        ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

        // LOGICAL FIX: Removed the while(true) loop from app_main.
        // app_main is for initialization. The FreeRTOS tasks (like 
        // command_runner_task and the BT stack) will run in the background.
        ESP_LOGI(SPP_TAG, "Initialization complete. Rover is ready for BT commands.");
    }
}
