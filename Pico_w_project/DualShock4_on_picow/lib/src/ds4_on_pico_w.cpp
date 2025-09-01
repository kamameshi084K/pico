/**
 * @file ds4_on_pico_w.cpp
 * @version 0.2.0
 * @date 2024-12-31
 *
 * @copyright Copyright (c) 2024 @Akari.
 *
 */
#include "ds4_on_pico_w.hpp"

// config
#include "btstack_config.h"
// bluetooth
#include "btstack.h"
#include "btstack_run_loop.h"
#include "classic/sdp_server.h"
#include "pico/async_context.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

///////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////
#pragma region Ds4forPicoW_Definitions

#define DS4_FOR_PICO_W_LOG_HEADER "[DOP] "

#define MAX_ATTRIBUTE_VALUE_SIZE 512
#define MAX_DEVICES              20
#define INQUIRY_INTERVAL         1

#pragma endregion

///////////////////////////////////////////////////////
// Structures
///////////////////////////////////////////////////////
#pragma region Ds4forPicoW_Structures

enum DEVICE_STATE
{
    REMOTE_NAME_REQUEST,
    REMOTE_NAME_INQUIRED,
    REMOTE_NAME_FETCHED
};
struct device {
    bd_addr_t address;
    uint8_t pageScanRepetitionMode;
    uint16_t clockOffset;
    enum DEVICE_STATE state;
};
enum STATE
{
    INIT,
    ACTIVE
};
enum DEVICE_TYPE
{
    DEVICE_UNKNOWN,
    DEVICE_DS4,
    DEVICE_DS5
};

const struct DualShock4_state default_state = {
    .report_id = 0x0,
    .hat       = (hat_t)0x8,
    .share     = false,
    .options   = false,
    .ps        = false,
    .triangle  = false,
    .square    = false,
    .circle    = false,
    .cross     = false,
    .fn1       = false,
    .fn2       = false,
    .mute      = false,

    .touch             = false,
    .touch_packet_size = 0x00,
    .touch_timestamp   = 0x00,

    .touch_f1_active  = false,
    .touch_f1_counter = 0x00,
    .touch_f1_x       = 0x00,
    .touch_f1_y       = 0x00,

    .touch_f2_active  = false,
    .touch_f2_counter = 0x00,
    .touch_f2_x       = 0x00,
    .touch_f2_y       = 0x00,

    .touch_f1_spare_active  = false,
    .touch_f1_spare_counter = 0x00,
    .touch_f1_spare_x       = 0x00,
    .touch_f1_spare_y       = 0x00,

    .touch_f2_spare_active  = false,
    .touch_f2_spare_counter = 0x00,
    .touch_f2_spare_x       = 0x00,
    .touch_f2_spare_y       = 0x00,

    .l1              = false,
    .l2              = false,
    .l2_value        = 0x00,
    .r1              = false,
    .r2              = false,
    .r2_value        = 0x00,
    .l3              = false,
    .l3_x            = 0x00,
    .l3_y            = 0x00,
    .r3              = false,
    .r3_x            = 0x00,
    .r3_y            = 0x00,
    .gyro_x          = 0x00,
    .gyro_y          = 0x00,
    .gyro_z          = 0x00,
    .accel_x         = 0x00,
    .accel_y         = 0x00,
    .accel_z         = 0x00,
    .battery         = 0x00,
    .battery_level   = 0x00,
    .connected_usb   = false,
    .connected_mic   = false,
    .connected_phone = false,
    .timestamp       = 0x00,
    .linked          = false,
};

const char *WirelessController = "Wireless Controller";
struct __attribute__((packed)) input_report_ds4 {
    uint8_t dummy0[2];
    uint8_t report_id;   // byte[0]
    uint8_t lx, ly;      // byte[1], byte[2]
    uint8_t rx, ry;      // byte[3], byte[4]
    uint8_t buttons[3];  // byte[5], byte[6], byte[7]
    uint8_t l2, r2;      // byte[8], byte[9]
    uint16_t timestamp;  // byte[10-11]
    uint8_t battery;     // byte[12]
    int16_t accel[3];    // byte[13-14], byte[15-16], byte[17-18]
    int16_t gyro[3];     // byte[19-20], byte[21-22], byte[23-24]
    uint8_t dummy1[5];   // byte[25], byte[26], byte[27], byte[28], byte[29]
    uint8_t status;      // byte[30]
    uint8_t dummy2[2];   // byte[31], byte[32]
    uint8_t pad_size;    // byte[33]
    uint8_t pad_counter; // byte[34]
    uint8_t pad1a[4];
    uint8_t pad2a[4];
    uint8_t pad1b[4];
    uint8_t pad2b[4];
};
const char *DualSenseWirelessController     = "DualSense Wireless Controller";
const char *DualSenseEdgeWirelessController = "DualSense Edge Wireless Controller";
struct __attribute__((packed)) input_report_ds5 {
    uint8_t report_id;
    uint8_t dummy0;
    uint8_t lx, ly;
    uint8_t rx, ry;
    uint8_t l2, r2;
    uint8_t dummy1;
    uint8_t buttons[4];
    uint8_t dummy2[4];
    int16_t accel[3];
    int16_t gyro[3];
    uint32_t timestamp;
    uint8_t timestamp_a;
    uint8_t pad1a[4];
    uint8_t pad2a[4];
    uint8_t pad_counter;

    ///////////////////
    uint8_t battery;
    uint8_t dummy4[11];
    uint8_t status[2];
    uint8_t dummy5[12];

    /////////////////////////
};

#pragma endregion

///////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////
#pragma region Ds4forPicoW_Variables

static hid_protocol_mode_t hid_host_report_mode = HID_PROTOCOL_MODE_REPORT;
static bool hid_host_descriptor_available       = false;
static uint16_t hid_host_cid                    = 0;
static uint8_t hid_descriptor_storage[MAX_ATTRIBUTE_VALUE_SIZE];

static bd_addr_t remote_addr;
static bd_addr_t connected_addr;
static btstack_packet_callback_registration_t hci_event_callback_registration;

static char *remote_addr_string;
static int deviceCount = 0;
static bool hid_linked = false;

/////////////////
struct DualShock4_state latest;
struct DualShock4_state ds4_state;
struct DS4forPicoW::config request_config;

struct device devices[MAX_DEVICES];

/////////////////

enum STATE state             = INIT;
enum DEVICE_TYPE device_type = DEVICE_UNKNOWN;

#pragma endregion

///////////////////////////////////////////////////////
// Prototypes
///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
// Blink Timer
///////////////////////////////////////////////////////
#pragma region Ds4forPicoW_BlinkTimer

#define DS4_FOR_PICO_W_BLINK_MS 250
volatile bool g_flag_blink_led = false;
static btstack_timer_source_t blink_timer;
static int blink_timer_ms = DS4_FOR_PICO_W_BLINK_MS;
static void func_blink_handler(btstack_timer_source_t *ts)
{
    static bool on = 0;

    if (hid_host_cid != 0) {
        on = true;
    } else {
        on = !on;
    }

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, !!on);

    btstack_run_loop_set_timer(&blink_timer, blink_timer_ms);
    btstack_run_loop_add_timer(&blink_timer);
}
#pragma endregion

///////////////////////////////////////////////////////
// Scan Functions
///////////////////////////////////////////////////////
#pragma region Ds4forPicoW_ScanFunctions
static int func_get_device_index_for_address(bd_addr_t addr)
{
    int j;
    for (j = 0; j < deviceCount; j++) {
        if (bd_addr_cmp(addr, devices[j].address) == 0) {
            return j;
        }
    }
    return -1;
}

static void func_continue_remote_names(void)
{
    for (int i = 0; i < deviceCount; i++) {
        // remote name request
        if (devices[i].state == REMOTE_NAME_REQUEST) {
            devices[i].state = REMOTE_NAME_INQUIRED;
#if ENABLE_LOG_DEBUG
            printf("%sGet remote name of [%s]\n", DS4_FOR_PICO_W_LOG_HEADER, bd_addr_to_str(devices[i].address));
#endif
            gap_remote_name_request(devices[i].address, devices[i].pageScanRepetitionMode, devices[i].clockOffset | 0x8000);
            return;
        }
    }
    gap_inquiry_start(INQUIRY_INTERVAL);
}

char *func_get_mac(uint8_t packet_type, uint8_t *packet, uint8_t event)
{
    bd_addr_t addr;
    int i;
    int index;

    char *mac = (char *)"";

    switch (state) {
        case INIT: {
            switch (event) {
                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                        gap_inquiry_start(INQUIRY_INTERVAL);
                        state = ACTIVE;
                    }
                    break;
                default:
                    break;
            }
        } break;

        case ACTIVE:
            switch (event) {
                case GAP_EVENT_INQUIRY_RESULT: {
                    if (deviceCount >= MAX_DEVICES)
                        break; // already full
                    gap_event_inquiry_result_get_bd_addr(packet, addr);
                    index = func_get_device_index_for_address(addr);
                    if (index >= 0) {
                        break; // already in our list
                    }
                    memcpy(devices[deviceCount].address, addr, 6);
                    devices[deviceCount].pageScanRepetitionMode = gap_event_inquiry_result_get_page_scan_repetition_mode(packet);
                    devices[deviceCount].clockOffset            = gap_event_inquiry_result_get_clock_offset(packet);
                    char *mac_addr                              = bd_addr_to_str(addr);
#if ENABLE_LOG_DEBUG
                    printf("%s[%s] Device found: [%s] ", DS4_FOR_PICO_W_LOG_HEADER, "GAP_EVENT_INQUIRY_RESULT", mac_addr);
                    printf("with COD[0x%06X], ", (unsigned int)gap_event_inquiry_result_get_class_of_device(packet));
                    printf("pageScan[%d], ", devices[deviceCount].pageScanRepetitionMode);
                    printf("clock offset[0x%04X]", devices[deviceCount].clockOffset);
#endif
                    if (gap_event_inquiry_result_get_rssi_available(packet)) {
#if ENABLE_LOG_DEBUG
                        printf(", rssi %d dBm", (int8_t)gap_event_inquiry_result_get_rssi(packet));
#endif
                    }
                    if (gap_event_inquiry_result_get_name_available(packet)) {
                        bool flag_check = true;
                        char name_buffer[240];
                        int name_len = gap_event_inquiry_result_get_name_len(packet);
                        memcpy(name_buffer, gap_event_inquiry_result_get_name(packet), name_len);
                        name_buffer[name_len] = 0;
#if ENABLE_LOG_DEBUG
                        printf(", name '%s'", name_buffer);
#endif
                        devices[deviceCount].state = REMOTE_NAME_FETCHED;
                        if (strcmp("", request_config.mac_address.c_str()) != 0) {
                            if (strcmp(mac_addr, request_config.mac_address.c_str()) != 0) {
                                flag_check = false;
                            }
                        }
                        if (true == flag_check) {
                            if (strcmp(name_buffer, WirelessController) == 0) {
                                mac         = mac_addr;
                                device_type = DEVICE_DS4;
                            } else if (strcmp(name_buffer, DualSenseWirelessController) == 0) {
                                mac         = mac_addr;
                                device_type = DEVICE_DS5;
                            } else if (strcmp(name_buffer, DualSenseEdgeWirelessController) == 0) {
                                mac         = mac_addr;
                                device_type = DEVICE_DS5;
                            }
                        } else {
#if ENABLE_LOG_DEBUG
                            printf("%sNot match device name: '%s'!='%s'\n", DS4_FOR_PICO_W_LOG_HEADER, request_config.mac_address.c_str(), mac_addr);
#endif
                        }
                    } else {
                        devices[deviceCount].state = REMOTE_NAME_REQUEST;
                    }
#if ENABLE_LOG_DEBUG
                    printf("\n");
#endif
                    deviceCount++;
                } break;

                case GAP_EVENT_INQUIRY_COMPLETE:
                    for (i = 0; i < deviceCount; i++) {
                        // retry remote name request
                        if (devices[i].state == REMOTE_NAME_INQUIRED)
                            devices[i].state = REMOTE_NAME_REQUEST;
                    }
                    func_continue_remote_names();
                    break;

                case HCI_EVENT_REMOTE_NAME_REQUEST_COMPLETE: {
                    bool flag_check = true;
                    reverse_bd_addr(&packet[3], addr);
                    char *mac_addr = bd_addr_to_str(addr);
                    index          = func_get_device_index_for_address(addr);
                    if (index >= 0) {
                        if (packet[2] == 0) {
#if ENABLE_LOG_INFO
                            printf("%sDevice Name: '%s'\n", DS4_FOR_PICO_W_LOG_HEADER, &packet[9]);
#endif
                            devices[index].state = REMOTE_NAME_FETCHED;
                            if (strcmp("", request_config.mac_address.c_str()) != 0) {
                                if (strcmp(mac_addr, request_config.mac_address.c_str()) != 0) {
                                    flag_check = false;
                                }
                            }
                            if (true == flag_check) {
                                if (strcmp((char const *)&packet[9], WirelessController) == 0) {
                                    mac         = mac_addr;
                                    device_type = DEVICE_DS4;
                                } else if (strcmp((char const *)&packet[9], DualSenseWirelessController) == 0) {
                                    mac         = mac_addr;
                                    device_type = DEVICE_DS5;
                                } else if (strcmp((char const *)&packet[9], DualSenseEdgeWirelessController) == 0) {
                                    mac         = mac_addr;
                                    device_type = DEVICE_DS5;
                                }
                            } else {
#if ENABLE_LOG_DEBUG
                                printf("%sNot match device name: '%s'!='%s'\n", DS4_FOR_PICO_W_LOG_HEADER, request_config.mac_address.c_str(), mac_addr);
#endif
                            }
                        } else {
#if ENABLE_LOG_ERROR
                            printf("%sFailed to get name: page timeout\n", DS4_FOR_PICO_W_LOG_HEADER);
#endif
                        }
                    }
                    func_continue_remote_names();
                } break;

                default:
                    break;
            }
            break;

        default:
            break;
    }

    return mac;
}
#pragma endregion

///////////////////////////////////////////////////////
// Bluetooth HID functions
///////////////////////////////////////////////////////
#pragma region Ds4forPicoW_BluetoothHIDFunctions

static void func_hid_host_handle_interrupt_report(const uint8_t *packet, uint16_t packet_len)
{
    if (packet_len < 1) {
        return;
    }
    // Only interested in report_id 0x11
    if (DEVICE_DS4 == device_type) {
        if ((packet[0] != 0xa1) || (packet[1] != 0x11)) {
            return;
        }
        if (packet_len < sizeof(struct input_report_ds4) + 1) {
            return;
        }

        struct input_report_ds4 *report = (struct input_report_ds4 *)&packet[1];

        // Note: This assumes that we're protected by async_context's single-threaded-ness
        latest = (struct DualShock4_state){
            .report_id = (uint8_t)(report->report_id),
            .hat       = (hat_t)(report->buttons[0] & 0x0Fu),
            .share     = (bool)(report->buttons[1] & 0x10u),
            .options   = (bool)(report->buttons[1] & 0x20u),
            .ps        = (bool)(report->buttons[2] & 0x01u),
            .triangle  = (bool)(report->buttons[0] & 0x80u),
            .square    = (bool)(report->buttons[0] & 0x10u),
            .circle    = (bool)(report->buttons[0] & 0x40u),
            .cross     = (bool)(report->buttons[0] & 0x20u),
            .fn1       = false,
            .fn2       = false,
            .mute      = false,
            .touch     = (bool)(report->buttons[2] & 0x02u),

            .touch_packet_size = report->pad_size,
            .touch_timestamp   = report->pad_counter,

            .touch_f1_active  = !(bool)(report->pad1a[0] & 0x80u),
            .touch_f1_counter = (uint8_t)(report->pad1a[0] & 0x7Fu),
            .touch_f1_x       = (uint16_t)((report->pad1a[1]) | ((report->pad1a[2] & 0x0Fu) << 8)),
            .touch_f1_y       = (uint16_t)(((uint16_t)(report->pad1a[3]) << 4) | ((report->pad1a[2] & 0xF0u) >> 4)),

            .touch_f2_active  = !(bool)(report->pad2a[0] & 0x80u),
            .touch_f2_counter = (uint8_t)(report->pad2a[0] & 0x7Fu),
            .touch_f2_x       = (uint16_t)((report->pad2a[1]) | ((report->pad2a[2] & 0x0Fu) << 8)),
            .touch_f2_y       = (uint16_t)(((uint16_t)(report->pad2a[3]) << 4) | ((report->pad2a[2] & 0xF0u) >> 4)),

            .touch_f1_spare_active  = !(bool)(report->pad1b[0] & 0x80u),
            .touch_f1_spare_counter = (uint8_t)(report->pad1b[0] & 0x7Fu),
            .touch_f1_spare_x       = (uint16_t)((report->pad1b[1]) | ((report->pad1b[2] & 0x0Fu) << 8)),
            .touch_f1_spare_y       = (uint16_t)(((uint16_t)(report->pad1b[3]) << 4) | ((report->pad1b[2] & 0xF0u) >> 4)),

            .touch_f2_spare_active  = !(bool)(report->pad2b[0] & 0x80u),
            .touch_f2_spare_counter = (uint8_t)(report->pad2b[0] & 0x7Fu),
            .touch_f2_spare_x       = (uint16_t)((report->pad2b[1]) | ((report->pad2b[2] & 0x0Fu) << 8)),
            .touch_f2_spare_y       = (uint16_t)(((uint16_t)(report->pad2b[3]) << 4) | ((report->pad2b[2] & 0xF0u) >> 4)),

            .l1              = (bool)(report->buttons[1] & 0x01u),
            .l2              = (bool)(report->buttons[1] & 0x04u),
            .l2_value        = report->l2,
            .r1              = (bool)(report->buttons[1] & 0x02u),
            .r2              = (bool)(report->buttons[1] & 0x08u),
            .r2_value        = report->r2,
            .l3              = (bool)(report->buttons[1] & 0x40u),
            .l3_x            = report->lx,
            .l3_y            = report->ly,
            .r3              = (bool)(report->buttons[1] & 0x80u),
            .r3_x            = report->rx,
            .r3_y            = report->ry,
            .gyro_x          = (int16_t)report->gyro[0],
            .gyro_y          = (int16_t)report->gyro[1],
            .gyro_z          = (int16_t)report->gyro[2],
            .accel_x         = (int16_t)report->accel[0],
            .accel_y         = (int16_t)report->accel[1],
            .accel_z         = (int16_t)report->accel[2],
            .battery         = report->battery,
            .battery_level   = (uint8_t)(report->status & 0x0Fu),
            .connected_usb   = (report->status & 0x10u) > 0 ? true : false,
            .connected_mic   = (report->status & 0x40u) > 0 ? true : false,
            .connected_phone = (report->status & 0x20u) > 0 ? true : false,
            .timestamp       = (uint32_t)report->timestamp,
            .linked          = hid_linked,
        };
    } else if (DEVICE_DS5 == device_type) {
        if ((packet[0] != 0xa1) || (packet[1] != 0x31)) {
            return;
        }
        if (packet_len < sizeof(struct input_report_ds5) + 1) {
            return;
        }

        struct input_report_ds5 *report = (struct input_report_ds5 *)&packet[1];

        // Note: This assumes that we're protected by async_context's single-threaded-ness
        latest = (struct DualShock4_state){
            .report_id = (uint8_t)(report->report_id),
            .hat       = (hat_t)(report->buttons[0] & 0x0Fu),
            .share     = (bool)(report->buttons[1] & 0x10u),
            .options   = (bool)(report->buttons[1] & 0x20u),
            .ps        = (bool)(report->buttons[2] & 0x01u),
            .triangle  = (bool)(report->buttons[0] & 0x80u),
            .square    = (bool)(report->buttons[0] & 0x10u),
            .circle    = (bool)(report->buttons[0] & 0x40u),
            .cross     = (bool)(report->buttons[0] & 0x20u),
            .fn1       = (bool)(report->buttons[2] & 0x10u),
            .fn2       = (bool)(report->buttons[2] & 0x20u),
            .mute      = (bool)(report->buttons[2] & 0x04u),

            .touch             = (bool)(report->buttons[2] & 0x02u),
            .touch_packet_size = 0, // report->pad_size,
            .touch_timestamp   = report->pad_counter,

            .touch_f1_active  = !(bool)(report->pad1a[0] & 0x80u),
            .touch_f1_counter = (uint8_t)(report->pad1a[0] & 0x7Fu),
            .touch_f1_x       = (uint16_t)((report->pad1a[1]) | ((report->pad1a[2] & 0x0Fu) << 8)),
            .touch_f1_y       = (uint16_t)(((uint16_t)(report->pad1a[3]) << 4) | ((report->pad1a[2] & 0xF0u) >> 4)),

            .touch_f2_active  = !(bool)(report->pad2a[0] & 0x80u),
            .touch_f2_counter = (uint8_t)(report->pad2a[0] & 0x7Fu),
            .touch_f2_x       = (uint16_t)((report->pad2a[1]) | ((report->pad2a[2] & 0x0Fu) << 8)),
            .touch_f2_y       = (uint16_t)(((uint16_t)(report->pad2a[3]) << 4) | ((report->pad2a[2] & 0xF0u) >> 4)),

            .touch_f1_spare_active  = false,
            .touch_f1_spare_counter = 0,
            .touch_f1_spare_x       = 0,
            .touch_f1_spare_y       = 0,

            .touch_f2_spare_active  = false,
            .touch_f2_spare_counter = 0,
            .touch_f2_spare_x       = 0,
            .touch_f2_spare_y       = 0,

            .l1              = (bool)(report->buttons[1] & 0x01u),
            .l2              = (bool)(report->buttons[1] & 0x04u),
            .l2_value        = report->l2,
            .r1              = (bool)(report->buttons[1] & 0x02u),
            .r2              = (bool)(report->buttons[1] & 0x08u),
            .r2_value        = report->r2,
            .l3              = (bool)(report->buttons[1] & 0x40u),
            .l3_x            = report->lx,
            .l3_y            = report->ly,
            .r3              = (bool)(report->buttons[1] & 0x80u),
            .r3_x            = report->rx,
            .r3_y            = report->ry,
            .gyro_x          = (int16_t)report->gyro[0],
            .gyro_y          = (int16_t)report->gyro[1],
            .gyro_z          = (int16_t)report->gyro[2],
            .accel_x         = (int16_t)report->accel[0],
            .accel_y         = (int16_t)report->accel[1],
            .accel_z         = (int16_t)report->accel[2],
            .battery         = report->battery,
            .battery_level   = (uint8_t)(report->battery & 0x0Fu),
            .connected_usb   = (report->status[0] & 0x10u) > 0 ? true : false,
            .connected_mic   = (report->status[0] & 0x02u) > 0 ? true : false,
            .connected_phone = (report->status[0] & 0x01u) > 0 ? true : false,
            .timestamp       = (uint32_t)report->timestamp,
            .linked          = hid_linked,
        };
    }
}

bool func_bt_hid_get_latest(struct DualShock4_state *dst)
{
#if 0
    static async_context_t *context = cyw43_arch_async_context();
    async_context_acquire_lock_blocking(context);
    memcpy(dst, &latest, sizeof(*dst));
    async_context_release_lock(context);
    return true;
#else
    memcpy(dst, &latest, sizeof(*dst));
    return true;
#endif
}

static void func_bt_hid_disconnected(bd_addr_t addr)
{
    hid_host_cid                  = 0;
    hid_host_descriptor_available = false;

    memcpy(&latest, &default_state, sizeof(latest));
    latest.linked = hid_linked;
}

static void func_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    UNUSED(channel);
    UNUSED(size);

    uint8_t event;
    bd_addr_t event_addr;
    uint8_t status;

    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }

    event = hci_event_packet_get_type(packet);

    if (remote_addr_string[0] == '\0') {
        remote_addr_string = func_get_mac(packet_type, packet, event);
        if (remote_addr_string[0] != '\0') {
            sscanf_bd_addr(remote_addr_string, remote_addr);
            func_bt_hid_disconnected(remote_addr);
            status = hid_host_connect(remote_addr, hid_host_report_mode, &hid_host_cid);
#if ENABLE_LOG_ERROR
            if (status != ERROR_CODE_SUCCESS) {
                printf("%sHID_HOST_CONNECT command failed: 0x%02x\n", DS4_FOR_PICO_W_LOG_HEADER, status);
            }
#endif
            return;
        } else {
            return;
        }
    }

    switch (event) {
        case BTSTACK_EVENT_STATE:
            // On boot, we try a manual connection
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                status = hid_host_connect(remote_addr, hid_host_report_mode, &hid_host_cid);
                if (status != ERROR_CODE_SUCCESS) {
#if ENABLE_LOG_ERROR
                    printf("%sHID_HOST_CONNECT command failed: 0x%02x\n", DS4_FOR_PICO_W_LOG_HEADER, status);
#endif
                    return;
                }
#if ENABLE_LOG_DEBUG
                printf("%sStarting HID_HOST_CONNECT (%s)\n", DS4_FOR_PICO_W_LOG_HEADER, bd_addr_to_str(remote_addr));
#endif
            }
            break;
        case HCI_EVENT_CONNECTION_COMPLETE:
#if ENABLE_LOG_DEBUG
            status = hci_event_connection_complete_get_status(packet);
            printf("%sConnection complete: %X\n", DS4_FOR_PICO_W_LOG_HEADER, status);
#endif
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE: {
#if ENABLE_LOG_INFO
            status         = hci_event_disconnection_complete_get_status(packet);
            uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
            printf("%sDisconnection complete: status: 0x%X, reason: 0x%X\n", DS4_FOR_PICO_W_LOG_HEADER, status, reason);
#endif
            hid_linked = false;
            memcpy(&latest, &default_state, sizeof(latest));
        } break;
        case HCI_EVENT_MAX_SLOTS_CHANGED:
#if ENABLE_LOG_DEBUG
            status = hci_event_max_slots_changed_get_lmp_max_slots(packet);
            printf("%sMax slots changed: %X\n", DS4_FOR_PICO_W_LOG_HEADER, status);
#endif
            break;
        case HCI_EVENT_PIN_CODE_REQUEST:
#if ENABLE_LOG_DEBUG
            printf("%sPin code request. Responding '0000'\n", DS4_FOR_PICO_W_LOG_HEADER);
#endif
            hci_event_pin_code_request_get_bd_addr(packet, event_addr);
            gap_pin_code_response(event_addr, "0000");
            break;
        case HCI_EVENT_USER_CONFIRMATION_REQUEST:
#if ENABLE_LOG_DEBUG
            printf("%sSSP User Confirmation Request: %d\n", DS4_FOR_PICO_W_LOG_HEADER, little_endian_read_32(packet, 8));
#endif
            break;
        case HCI_EVENT_HID_META: {
            uint8_t hid_event = hci_event_hid_meta_get_subevent_code(packet);
            switch (hid_event) {
                case HID_SUBEVENT_INCOMING_CONNECTION:
                    hid_subevent_incoming_connection_get_address(packet, event_addr);
#if ENABLE_LOG_DEBUG
                    printf("%sAccepting connection from %s\n", DS4_FOR_PICO_W_LOG_HEADER, bd_addr_to_str(event_addr));
#endif
                    hid_host_accept_connection(hid_subevent_incoming_connection_get_hid_cid(packet), hid_host_report_mode);
                    break;
                case HID_SUBEVENT_CONNECTION_OPENED:
                    status = hid_subevent_connection_opened_get_status(packet);
                    hid_subevent_connection_opened_get_bd_addr(packet, event_addr);
                    if (status != ERROR_CODE_SUCCESS) {
#if ENABLE_LOG_ERROR
                        printf("%sConnection to %s failed: 0x%02x\n", DS4_FOR_PICO_W_LOG_HEADER, bd_addr_to_str(event_addr), status);
#endif
                        func_bt_hid_disconnected(event_addr);
                        return;
                    }
                    hid_host_descriptor_available = false;
                    hid_host_cid                  = hid_subevent_connection_opened_get_hid_cid(packet);
#if ENABLE_LOG_INFO
                    printf("%sConnected to (%s)\n", DS4_FOR_PICO_W_LOG_HEADER, bd_addr_to_str(event_addr));
#endif
                    bd_addr_copy(connected_addr, event_addr);
                    hid_linked     = true;
                    blink_timer_ms = request_config.blink_time_ms_rescan;
                    break;
                case HID_SUBEVENT_DESCRIPTOR_AVAILABLE:
                    status = hid_subevent_descriptor_available_get_status(packet);
                    if (status == ERROR_CODE_SUCCESS) {
                        hid_host_descriptor_available = true;

                        uint16_t dlen = hid_descriptor_storage_get_descriptor_len(hid_host_cid);
#if ENABLE_LOG_DEBUG
                        printf("%sHID descriptor available. Len: %d\n", DS4_FOR_PICO_W_LOG_HEADER, dlen);
#endif

                        // Send FEATURE 0x05, to switch the controller to "full" report mode
                        hid_host_send_get_report(hid_host_cid, HID_REPORT_TYPE_FEATURE, 0x05);
                    } else {
#if ENABLE_LOG_ERROR
                        printf("%sCouldn't process HID Descriptor, status: %d\n", DS4_FOR_PICO_W_LOG_HEADER, status);
#endif
                    }
                    break;
                case HID_SUBEVENT_REPORT:
                    if (hid_host_descriptor_available) {
                        func_hid_host_handle_interrupt_report(hid_subevent_report_get_report(packet), hid_subevent_report_get_report_len(packet));
                    } else {
#if ENABLE_LOG_ERROR
                        printf("%sNo hid host descriptor available\n", DS4_FOR_PICO_W_LOG_HEADER);
                        // printf_hexdump(hid_subevent_report_get_report(packet), hid_subevent_report_get_report_len(packet));
#endif
                    }
                    break;
                case HID_SUBEVENT_SET_PROTOCOL_RESPONSE: {
                    status = hid_subevent_set_protocol_response_get_handshake_status(packet);
                    if (status != HID_HANDSHAKE_PARAM_TYPE_SUCCESSFUL) {
#if ENABLE_LOG_ERROR
                        printf("%sProtocol handshake error: 0x%02x\n", DS4_FOR_PICO_W_LOG_HEADER, status);
#endif
                        break;
                    }
                    hid_protocol_mode_t proto = static_cast<hid_protocol_mode_t>(hid_subevent_set_protocol_response_get_protocol_mode(packet));
                    switch (proto) {
                        case HID_PROTOCOL_MODE_BOOT:
#if ENABLE_LOG_DEBUG
                            printf("%sNegotiated protocol: BOOT\n", DS4_FOR_PICO_W_LOG_HEADER);
#endif
                            break;
                        case HID_PROTOCOL_MODE_REPORT:
#if ENABLE_LOG_DEBUG
                            printf("%sNegotiated protocol: REPORT\n", DS4_FOR_PICO_W_LOG_HEADER);
#endif
                            break;
                        default:
#if ENABLE_LOG_ERROR
                            printf("%sNegotiated unknown protocol: 0x%X\n", DS4_FOR_PICO_W_LOG_HEADER, proto);
#endif
                            break;
                    }
                } break;
                case HID_SUBEVENT_CONNECTION_CLOSED:
#if ENABLE_LOG_DEBUG
                    printf("%sHID connection closed: %s\n", DS4_FOR_PICO_W_LOG_HEADER, bd_addr_to_str(connected_addr));
#endif
                    func_bt_hid_disconnected(connected_addr);
                    break;
                case HID_SUBEVENT_GET_REPORT_RESPONSE: {
                    status        = hid_subevent_get_report_response_get_handshake_status(packet);
                    uint16_t dlen = hid_subevent_get_report_response_get_report_len(packet);
#if ENABLE_LOG_DEBUG
                    printf("%sGET_REPORT response. status: %d, len: %d\n", DS4_FOR_PICO_W_LOG_HEADER, status, dlen);
#endif
                } break;
                default:
#if ENABLE_LOG_DEBUG
                    printf("%sUnknown HID subevent: 0x%X\n", DS4_FOR_PICO_W_LOG_HEADER, hid_event);
#endif
                    break;
            }
        } break;
        default:
            // printf("Unknown HCI event: 0x%X\n", event);
            break;
    }
}

#pragma endregion

///////////////////////////////////////////////////////
// Main functions
///////////////////////////////////////////////////////
#pragma region MainFunctions

void func_bt_hid_main()
{
    if (cyw43_arch_init()) {
        return;
    }

    gap_set_security_level(LEVEL_2);

    if (true == g_flag_blink_led) {
        blink_timer.process = &func_blink_handler;
        blink_timer_ms      = request_config.blink_time_ms_search;
        btstack_run_loop_set_timer(&blink_timer, blink_timer_ms);
        btstack_run_loop_add_timer(&blink_timer);
    }

    // Initialize L2CAP
    l2cap_init();

    sdp_init();

    // Initialize HID Host
    hid_host_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));
    hid_host_register_packet_handler(func_packet_handler);

    // Allow sniff mode requests by HID device and support role switch
    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_SNIFF_MODE | LM_LINK_POLICY_ENABLE_ROLE_SWITCH);

    // try to become master on incoming connections
    hci_set_master_slave_policy(HCI_ROLE_MASTER);

    // register for HCI events
    hci_event_callback_registration.callback = &func_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    if (remote_addr_string[0] != '\0') {
        sscanf_bd_addr(remote_addr_string, remote_addr);
        func_bt_hid_disconnected(remote_addr);
    } else {
        sscanf_bd_addr("00:00:00:00:00:00", remote_addr);
        func_bt_hid_disconnected(remote_addr);
    }
    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();
    hci_power_control(HCI_POWER_OFF);

    if (true == g_flag_blink_led) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
        btstack_run_loop_remove_timer(&blink_timer);
    }

    sdp_deinit();
    sm_deinit();
}
void func_bt_closing()
{
    btstack_run_loop_trigger_exit();
}
#pragma endregion

///////////////////////////////////////////////////////
// Class DS4forPicoW
///////////////////////////////////////////////////////
#pragma region Class_DS4forPicoW
static bool DS4forPicoW_flag_setup = false;

DS4forPicoW::DS4forPicoW()
{
    memcpy(&ds4_state, &default_state, sizeof(ds4_state));
}
DS4forPicoW::~DS4forPicoW()
{
    if (true == DS4forPicoW_flag_setup) {
        multicore_launch_core1(func_bt_closing);
        multicore_reset_core1();
        DS4forPicoW_flag_setup = false;
    }
}
void DS4forPicoW::setup(config config)
{
    request_config = (struct DS4forPicoW::config){
        .mac_address          = (strcmp("", config.mac_address.c_str()) != 0) ? config.mac_address : "",
        .blink_led            = config.blink_led,
        .blink_time_ms_search = config.blink_time_ms_search,
        .blink_time_ms_rescan = config.blink_time_ms_rescan,
    };
    if (false == DS4forPicoW_flag_setup) {
        DS4forPicoW_flag_setup = true;
        g_flag_blink_led       = config.blink_led;
        multicore_launch_core1(func_bt_hid_main);
    }
}

DualShock4_state DS4forPicoW::get_state()
{
    func_bt_hid_get_latest(&ds4_state);
    return ds4_state;
}

char *DS4forPicoW::get_mac_address()
{
    if (remote_addr_string[0] == '\0') {
        return (char *)"";
    } else {
        return remote_addr_string;
    }
}

#pragma endregion
