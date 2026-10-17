#include "msp.h"
#include "ble_msp.h"
#include <string.h>
#include <stdio.h>

// MSP Message IDs
#define MSP_API_VERSION     1
#define MSP_FC_VARIANT      2
#define MSP_FC_VERSION      3
#define MSP_BOARD_INFO      4
#define MSP_BUILD_INFO      5
#define MSP_NAME            11
#define MSP_STATUS          101
#define MSP_RAW_IMU         102
#define MSP_MOTOR           104
#define MSP_RC              105
#define MSP_ATTITUDE        108
#define MSP_ALTITUDE        109
#define MSP_ANALOG          110
#define MSP_PID             112
#define MSP_SET_PID         202
#define MSP_SET_MOTOR       214
#define MSP_EEPROM_WRITE    250

typedef enum {
    MSP_IDLE,
    MSP_HEADER_M,
    MSP_HEADER_DIR,
    MSP_PAYLOAD_LEN,
    MSP_CMD,
    MSP_PAYLOAD,
    MSP_CHECKSUM
} msp_state_t;

static msp_state_t state = MSP_IDLE;
static uint8_t in_cmd = 0;
static uint8_t in_len = 0;
static uint8_t in_buf[128];
static uint8_t in_idx = 0;
static uint8_t in_checksum = 0;

static msp_telemetry_t current_telem;

// PID Defaults for 1104 Brushless Motors on 3S
static uint8_t pid_gains[3][3] = {
    { 38, 30, 23 }, // Roll  (P, I, D)
    { 42, 33, 26 }, // Pitch (P, I, D)
    { 45, 45, 0  }  // Yaw   (P, I, D)
};

static void msp_send_packet(uint8_t cmd, const uint8_t *payload, uint8_t len) {
    uint8_t out[128];
    uint8_t idx = 0;

    out[idx++] = '$';
    out[idx++] = 'M';
    out[idx++] = '>'; // Output to host
    out[idx++] = len;
    out[idx++] = cmd;

    uint8_t checksum = len ^ cmd;
    for (int i = 0; i < len; i++) {
        out[idx++] = payload[i];
        checksum ^= payload[i];
    }
    out[idx++] = checksum;

    // 1. Stream over USB-CDC
    fwrite(out, 1, idx, stdout);
    fflush(stdout);

    // 2. Stream simultaneously over Bluetooth BLE for wireless Betaflight Configurator
    ble_msp_send(out, idx);
}

void msp_init(void) {
    state = MSP_IDLE;
    memset(&current_telem, 0, sizeof(current_telem));
    current_telem.cycle_time_us = 1000;
    for (int i = 0; i < 4; i++) current_telem.motor[i] = 1000;
    for (int i = 0; i < 14; i++) current_telem.rc_channels[i] = 1500;
    current_telem.rc_channels[2] = 1000; // Throttle
}

void msp_update_telemetry(const msp_telemetry_t *telem) {
    if (telem) {
        memcpy(&current_telem, telem, sizeof(current_telem));
    }
}

static void msp_dispatch(uint8_t cmd) {
    uint8_t buf[64];
    memset(buf, 0, sizeof(buf));

    switch (cmd) {
        case MSP_API_VERSION:
            buf[0] = 2; // MSP protocol version
            buf[1] = 1; // API major
            buf[2] = 45; // API minor (Betaflight 4.5)
            msp_send_packet(cmd, buf, 3);
            break;

        case MSP_FC_VARIANT:
            memcpy(buf, "BTFL", 4); // Report as Betaflight
            msp_send_packet(cmd, buf, 4);
            break;

        case MSP_FC_VERSION:
            buf[0] = 4; // Major
            buf[1] = 5; // Minor
            buf[2] = 0; // Patch
            msp_send_packet(cmd, buf, 3);
            break;

        case MSP_BOARD_INFO: {
            // Target board identifier: "OSUX"
            memcpy(buf, "OSUX", 4);
            buf[4] = 0; // Hardware revision
            buf[5] = 0;
            // Target Board Name: "OsusX"
            const char *board_name = "OsusX";
            uint8_t name_len = strlen(board_name);
            buf[6] = name_len;
            memcpy(&buf[7], board_name, name_len);
            msp_send_packet(cmd, buf, 7 + name_len);
            break;
        }

        case MSP_BUILD_INFO: {
            const char *build_str = "OsusX-v1.0.0-ESP32S3";
            uint8_t len = strlen(build_str);
            memcpy(buf, build_str, len);
            msp_send_packet(cmd, buf, len);
            break;
        }

        case MSP_NAME: {
            // Craft Name displayed in Betaflight Configurator top bar
            const char *craft_name = "OsusX";
            uint8_t len = strlen(craft_name);
            memcpy(buf, craft_name, len);
            msp_send_packet(cmd, buf, len);
            break;
        }

        case MSP_STATUS: {
            // Report actual real hardware cycle time in microseconds
            uint16_t cycle_time = current_telem.cycle_time_us > 0 ? current_telem.cycle_time_us : 1000;
            uint16_t i2c_errs = 0;
            uint16_t sensor_flags = 0x01 | 0x02 | 0x04; // ACC | BARO | MAG
            uint32_t flight_mode = current_telem.armed ? 0x01 : 0x00;

            buf[0] = cycle_time & 0xFF; buf[1] = cycle_time >> 8;
            buf[2] = i2c_errs & 0xFF;   buf[3] = i2c_errs >> 8;
            buf[4] = sensor_flags & 0xFF; buf[5] = sensor_flags >> 8;
            buf[6] = flight_mode & 0xFF; buf[7] = (flight_mode >> 8) & 0xFF;
            buf[8] = (flight_mode >> 16) & 0xFF; buf[9] = (flight_mode >> 24) & 0xFF;
            buf[10] = 0; // Config profile
            msp_send_packet(cmd, buf, 11);
            break;
        }

        case MSP_RAW_IMU:
            for (int i = 0; i < 3; i++) {
                buf[i*2] = current_telem.acc[i] & 0xFF;
                buf[i*2 + 1] = current_telem.acc[i] >> 8;
                buf[6 + i*2] = current_telem.gyro[i] & 0xFF;
                buf[6 + i*2 + 1] = current_telem.gyro[i] >> 8;
                buf[12 + i*2] = current_telem.mag[i] & 0xFF;
                buf[12 + i*2 + 1] = current_telem.mag[i] >> 8;
            }
            msp_send_packet(cmd, buf, 18);
            break;

        case MSP_ATTITUDE:
            buf[0] = current_telem.roll_decideg & 0xFF;
            buf[1] = current_telem.roll_decideg >> 8;
            buf[2] = current_telem.pitch_decideg & 0xFF;
            buf[3] = current_telem.pitch_decideg >> 8;
            buf[4] = current_telem.yaw_decideg & 0xFF;
            buf[5] = current_telem.yaw_decideg >> 8;
            msp_send_packet(cmd, buf, 6);
            break;

        case MSP_ALTITUDE: {
            int32_t alt = current_telem.alt_cm;
            int16_t vario = current_telem.vario_cms;
            buf[0] = alt & 0xFF; buf[1] = (alt >> 8) & 0xFF;
            buf[2] = (alt >> 16) & 0xFF; buf[3] = (alt >> 24) & 0xFF;
            buf[4] = vario & 0xFF; buf[5] = vario >> 8;
            msp_send_packet(cmd, buf, 6);
            break;
        }

        case MSP_MOTOR:
            for (int i = 0; i < 4; i++) {
                buf[i*2] = current_telem.motor[i] & 0xFF;
                buf[i*2 + 1] = current_telem.motor[i] >> 8;
            }
            msp_send_packet(cmd, buf, 8);
            break;

        case MSP_RC:
            for (int i = 0; i < 8; i++) {
                buf[i*2] = current_telem.rc_channels[i] & 0xFF;
                buf[i*2 + 1] = current_telem.rc_channels[i] >> 8;
            }
            msp_send_packet(cmd, buf, 16);
            break;

        case MSP_PID:
            for (int i = 0; i < 3; i++) {
                buf[i*3] = pid_gains[i][0];
                buf[i*3 + 1] = pid_gains[i][1];
                buf[i*3 + 2] = pid_gains[i][2];
            }
            msp_send_packet(cmd, buf, 9);
            break;

        case MSP_SET_PID:
            if (in_len >= 9) {
                for (int i = 0; i < 3; i++) {
                    pid_gains[i][0] = in_buf[i*3];
                    pid_gains[i][1] = in_buf[i*3 + 1];
                    pid_gains[i][2] = in_buf[i*3 + 2];
                }
            }
            msp_send_packet(cmd, NULL, 0); // ACK
            break;

        case MSP_EEPROM_WRITE:
            msp_send_packet(cmd, NULL, 0); // ACK write
            break;

        default:
            msp_send_packet(cmd, NULL, 0); // Generic ACK
            break;
    }
}

void msp_process_byte(uint8_t c) {
    switch (state) {
        case MSP_IDLE:
            if (c == '$') state = MSP_HEADER_M;
            break;
        case MSP_HEADER_M:
            state = (c == 'M') ? MSP_HEADER_DIR : MSP_IDLE;
            break;
        case MSP_HEADER_DIR:
            state = (c == '<') ? MSP_PAYLOAD_LEN : MSP_IDLE; // '<' is host-to-FC
            break;
        case MSP_PAYLOAD_LEN:
            in_len = c;
            in_checksum = c;
            in_idx = 0;
            state = MSP_CMD;
            break;
        case MSP_CMD:
            in_cmd = c;
            in_checksum ^= c;
            if (in_len > 0) {
                state = MSP_PAYLOAD;
            } else {
                state = MSP_CHECKSUM;
            }
            break;
        case MSP_PAYLOAD:
            in_buf[in_idx++] = c;
            in_checksum ^= c;
            if (in_idx >= in_len) {
                state = MSP_CHECKSUM;
            }
            break;
        case MSP_CHECKSUM:
            if (c == in_checksum) {
                msp_dispatch(in_cmd);
            }
            state = MSP_IDLE;
            break;
    }
}
