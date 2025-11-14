// C++ headers
#include <cstdint>
#include <sys/types.h>

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
}

// Rover C++ headers
#include "motors.h"
#include "driver/ledc.h"

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
static DriveSystem *g_rover = nullptr;

// --- Constants for Rover Control ---
// You can adjust these values
#define ROVER_FORWARD_SPEED 1500
#define ROVER_BACKWARD_SPEED -1500 
#define ROVER_TURN_ANGLE 30.0f
#define ROVER_STOP_SPEED 0

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


    static void spp_read_handle(void * param)
    {
        static int16_t speed{0};
        static float angle{0.0f};

        int size = 0;
        int fd = (int)param;
        uint8_t *spp_data = NULL;

        // FIX: C++ requires an explicit cast from malloc's void*
        spp_data = (uint8_t *)malloc(SPP_DATA_LEN);
        if (!spp_data) {
            ESP_LOGE(SPP_TAG, "malloc spp_data failed, fd:%d", fd);
            goto done;
        }

        do {
            /* The frequency of calling this function also limits the speed at which the peer device can send data. */
            size = read(fd, spp_data, SPP_DATA_LEN);
            if (size < 0) {
                break;
            } else if (size == 0) {
                /* There is no data, retry after 500 ms */
                vTaskDelay(500 / portTICK_PERIOD_MS);
            } else {
                ESP_LOGI(SPP_TAG, "fd = %d data_len = %d", fd, size);
                ESP_LOG_BUFFER_HEX(SPP_TAG, spp_data, size);

                // Process all commands in the buffer, but only the last one
                // will be repeatedly executed.
                for (size_t i = 0; i < size; i++) {
                    
                    switch (spp_data[i])
                    {
                        case 0x46: // 'F'
                            if (speed < ROVER_FORWARD_SPEED) speed += 50;
                            g_rover->set_speed(speed);
                            break;
                        case 0x53: // 'L'
                            if (angle < 45.0f) angle += 3.0f;
                            g_rover->set_angle(angle);
                            break;
                        case 0x42: // 'B'
                            if (speed > ROVER_BACKWARD_SPEED) speed -= 50;
                            g_rover->set_speed(speed);
                            break;
                        case 0x43: // 'R'
                            if (angle > -45.0f) angle -= 3.0f;
                            g_rover->set_angle(angle);
                            break;
                        case 0x30: // '0'
                            speed = 0;
                            g_rover->set_speed(speed);
                            break;
                        
                        default:
                            g_rover->set(0, 0.0f);
                            ESP_LOGW(SPP_TAG, "Unknown command: 0x%02x", spp_data[i]);
                            break;
                    }
                }

                /* To avoid task watchdog */
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        } while (1);
    done:
        if (spp_data) {
            free(spp_data);
        }
        spp_wr_task_shut_down();
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
            g_rover->set(0, 0.0f);
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
        g_rover = new DriveSystem(g_pca9685_dev);
        ESP_LOGI(SPP_TAG, "Rover DriveSystem Initialized.");
        g_rover->set(0, 0.0f);

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
