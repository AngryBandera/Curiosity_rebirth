#include <cstring>
#include <iostream>
#include <uni.h>
#include <motors.h>
#include <cmath>
#include <math.h>

#define DEAD_ZONE 10
#define AXIS_MAX_INPUT 512.0f
#define POWER_EXPONENT 3.0f

static DriveSystem* g_rover = nullptr;


typedef struct my_platform_instance_s {
    uni_gamepad_seat_t gamepad_seat;
} my_platform_instance_t;

static void trigger_event_on_gamepad(uni_hid_device_t* d);
static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d);

static void my_platform_init(int argc, const char** argv) {
    logi("custom: init()\n");
}

static void my_platform_on_init_complete(void) {
    logi("custom: on_init_complete()\n");
    uni_bt_start_scanning_and_autoconnect_unsafe();
    uni_bt_allow_incoming_connections(true);
}

static uni_error_t my_platform_on_device_discovered(bd_addr_t addr, const char* name, uint16_t cod, uint8_t rssi) {
    if (((cod & UNI_BT_COD_MINOR_MASK) & UNI_BT_COD_MINOR_KEYBOARD) == UNI_BT_COD_MINOR_KEYBOARD) {
        logi("Ignoring keyboard\n");
        return UNI_ERROR_IGNORE_DEVICE;
    }

    return UNI_ERROR_SUCCESS;
}

static void my_platform_on_device_connected(uni_hid_device_t* d) {
    logi("custom: device connected: %p\n", d);
}

static void my_platform_on_device_disconnected(uni_hid_device_t* d) {
    logi("custom: device disconnected: %p\n", d);
    g_rover->set(0, 0.0f);
}

static uni_error_t my_platform_on_device_ready(uni_hid_device_t* d) {
    logi("custom: device ready: %p\n", d);
    my_platform_instance_t* ins = get_my_platform_instance(d);
    ins->gamepad_seat = GAMEPAD_SEAT_A;

    trigger_event_on_gamepad(d);
    return UNI_ERROR_SUCCESS;
}

static int32_t normalized_speed(int32_t y) {
    #define MAX_SPEED 4096

    if (abs(y) < DEAD_ZONE) return 0;

    float normalized_input = abs((float)y / AXIS_MAX_INPUT);
    
    float non_linear_scale = std::pow(normalized_input, POWER_EXPONENT);
    
    int sign = (y >= 0) ? -1 : 1;

    return (int32_t)(sign * non_linear_scale * MAX_SPEED);
}

static float normalized_angle(int32_t x) {
    #define MAX_ANGLE 30.0f
    #define MAX_ANGLE_RAW 512.0f

    if (std::abs(x) < DEAD_ZONE) return 0.0f;

    float normalized_input = abs((float)x / AXIS_MAX_INPUT);
    
    float non_linear_scale = std::pow(normalized_input, POWER_EXPONENT);
    
    int sign = (x >= 0) ? 1 : -1;

    return (float)(sign * non_linear_scale * MAX_ANGLE);
}
static void my_platform_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
    static uint8_t leds = 0;
    static uint8_t enabled = true;
    
    static uni_controller_t prev = {}; 
    
    uni_gamepad_t* gp;

    if (memcmp(&prev, ctl, sizeof(*ctl)) == 0) {
        return;
    }
    prev = *ctl;

    switch (ctl->klass) {
        case UNI_CONTROLLER_CLASS_GAMEPAD: {
            gp = &ctl->gamepad;
            int32_t speed = normalized_speed(gp->axis_y);
            float angle = normalized_angle(gp->axis_rx);
            
            if (gp->throttle > 450 && !g_rover->is_moving()) {
                g_rover->set(2000, 90);
            } else if (gp->brake > 450 && !g_rover->is_moving()) {
                g_rover->set(2000, -90);
            }
            else {
                g_rover->set(speed, angle);

                if ((abs(gp->axis_y) >= 450) && d->report_parser.play_dual_rumble != NULL) {
                    d->report_parser.play_dual_rumble(d, 0, 100, 255, 0);
                }
            }
            break;
        }
        case UNI_CONTROLLER_CLASS_NONE:
        default:
            break;
    }
}

static const uni_property_t* my_platform_get_property(uni_property_idx_t idx) {
    ARG_UNUSED(idx);
    return NULL;
}

static void my_platform_on_oob_event(uni_platform_oob_event_t event, void* data) {
    switch (event) {
        case UNI_PLATFORM_OOB_GAMEPAD_SYSTEM_BUTTON: {
            uni_hid_device_t* d = (uni_hid_device_t*)data;

            if (d == NULL) {
                loge("ERROR: my_platform_on_oob_event: Invalid NULL device\n");
                return;
            }
            logi("custom: on_device_oob_event(): %d\n", event);

            my_platform_instance_t* ins = get_my_platform_instance(d);
            ins->gamepad_seat = (ins->gamepad_seat == GAMEPAD_SEAT_A) ? GAMEPAD_SEAT_B : GAMEPAD_SEAT_A;

            trigger_event_on_gamepad(d);
            break;
        }
        case UNI_PLATFORM_OOB_BLUETOOTH_ENABLED:
            logi("custom: Bluetooth enabled: %d\n", (bool)(data));
            break;
        default:
            logi("my_platform_on_oob_event: unsupported event: 0x%04x\n", event);
            break;
    }
}

static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d) {
    return (my_platform_instance_t*)&d->platform_data[0];
}

static void trigger_event_on_gamepad(uni_hid_device_t* d) {
    my_platform_instance_t* ins = get_my_platform_instance(d);

    if (d->report_parser.play_dual_rumble != NULL) {
        d->report_parser.play_dual_rumble(d, 0, 150, 128, 40);
    }

    if (d->report_parser.set_player_leds != NULL) {
        d->report_parser.set_player_leds(d, ins->gamepad_seat);
    }

    if (d->report_parser.set_lightbar_color != NULL) {
        uint8_t red = (ins->gamepad_seat & 0x01) ? 0xff : 0;
        uint8_t green = (ins->gamepad_seat & 0x02) ? 0xff : 0;
        uint8_t blue = (ins->gamepad_seat & 0x04) ? 0xff : 0;
        d->report_parser.set_lightbar_color(d, red, green, blue);
    }
}

extern "C" struct uni_platform* get_my_platform(DriveSystem* ds) {
    static struct uni_platform plat = {};

    g_rover = ds;
    plat.name = "custom";
    plat.init = my_platform_init;
    plat.on_init_complete = my_platform_on_init_complete;
    plat.on_device_discovered = my_platform_on_device_discovered;
    plat.on_device_connected = my_platform_on_device_connected;
    plat.on_device_disconnected = my_platform_on_device_disconnected;
    plat.on_device_ready = my_platform_on_device_ready;
    plat.on_oob_event = my_platform_on_oob_event;
    plat.on_controller_data = my_platform_on_controller_data;
    plat.get_property = my_platform_get_property;

    return &plat;
}