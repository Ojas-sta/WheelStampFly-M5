#include "msp.h"
#include "ble_msp.h"
#include "hal/usb_serial_jtag_ll.h"
#include "esp_mac.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

// MSP Message IDs
#define MSP_API_VERSION     1
#define MSP_FC_VARIANT      2
#define MSP_FC_VERSION      3
#define MSP_BOARD_INFO      4
#define MSP_BUILD_INFO      5
#define MSP_NAME            11
#define MSP_FEATURE_CONFIG  36
#define MSP_MIXER_CONFIG    42
#define MSP_CF_SERIAL_CONFIG 54
#define MSP_ADVANCED_CONFIG 90
#define MSP_SENSOR_CONFIG    96
#define MSP_IDENT           100
#define MSP_STATUS          101
#define MSP_RAW_IMU         102
#define MSP_MOTOR           104
#define MSP_RC              105
#define MSP_ATTITUDE        108
#define MSP_ALTITUDE        109
#define MSP_ANALOG          110
#define MSP_RC_TUNING       111
#define MSP_PID             112
#define MSP_BOXNAMES        116
#define MSP_BOXIDS          119
#define MSP_SENSOR_ALIGNMENT 126
#define MSP_BATTERY_STATE   130
#define MSP_MOTOR_CONFIG    131
#define MSP_STATUS_EX       150
#define MSP_UID             160
#define MSP_SET_PID         202
#define MSP_SET_MOTOR       214
#define MSP_ACC_TRIM        240
#define MSP_EEPROM_WRITE    250

// MSP V2 Extended Commands
#define MSP2_MCU_INFO       0x300C

typedef enum {
    MSP_STATE_IDLE,
    MSP_STATE_HEADER_DOLLAR,
    MSP_STATE_HEADER_M, // MSP V1
    MSP_STATE_HEADER_X, // MSP V2
    // V1 states
    MSP_STATE_V1_DIR,
    MSP_STATE_V1_LEN,
    MSP_STATE_V1_CMD,
    MSP_STATE_V1_PAYLOAD,
    MSP_STATE_V1_CHECKSUM,
    // V2 states
    MSP_STATE_V2_DIR,
    MSP_STATE_V2_FLAG,
    MSP_STATE_V2_CMD_L,
    MSP_STATE_V2_CMD_H,
    MSP_STATE_V2_LEN_L,
    MSP_STATE_V2_LEN_H,
    MSP_STATE_V2_PAYLOAD,
    MSP_STATE_V2_CHECKSUM
} msp_parser_state_t;

static msp_parser_state_t state = MSP_STATE_IDLE;
static uint16_t in_cmd = 0;
static uint16_t in_len = 0;
static uint8_t in_buf[256];
static uint16_t in_idx = 0;
static uint8_t in_checksum = 0;
static uint8_t in_flag = 0;

static msp_telemetry_t current_telem;

// PID Defaults for 1104 Brushless Motors on 3S
static uint8_t pid_gains[3][3] = {
    { 38, 30, 23 }, // Roll  (P, I, D)
    { 42, 33, 26 }, // Pitch (P, I, D)
    { 45, 45, 0  }  // Yaw   (P, I, D)
};

// CRC8-DVB-S2 algorithm for MSP V2
static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t ch) {
    crc ^= ch;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

// Low-level hardware USB Serial/JTAG write with immediate flush
static void msp_usb_write(const uint8_t *data, size_t len) {
    if (!data || len == 0) return;
    size_t written = 0;
    while (written < len) {
        size_t chunk = len - written;
        if (chunk > 64) chunk = 64; // Max FIFO size is 64 bytes
        int timeout_us = 50000;
        while (!usb_serial_jtag_ll_txfifo_writable() && timeout_us > 0) {
            esp_rom_delay_us(100);
            timeout_us -= 100;
        }
        usb_serial_jtag_ll_write_txfifo(&data[written], chunk);
        usb_serial_jtag_ll_txfifo_flush(); // Force immediate USB transmission
        written += chunk;
    }
}

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

    // Direct hardware write and flush to USB Serial/JTAG
    msp_usb_write(out, idx);

    // Stream simultaneously over Bluetooth BLE
    ble_msp_send(out, idx);
}

static void msp2_send_packet(uint16_t cmd, const uint8_t *payload, uint16_t len) {
    uint8_t out[256];
    uint16_t idx = 0;

    out[idx++] = '$';
    out[idx++] = 'X';
    out[idx++] = '>';
    out[idx++] = 0; // flag: ok

    out[idx++] = cmd & 0xFF;
    out[idx++] = (cmd >> 8) & 0xFF;

    out[idx++] = len & 0xFF;
    out[idx++] = (len >> 8) & 0xFF;

    uint8_t crc = 0;
    crc = crc8_dvb_s2(crc, 0);
    crc = crc8_dvb_s2(crc, cmd & 0xFF);
    crc = crc8_dvb_s2(crc, (cmd >> 8) & 0xFF);
    crc = crc8_dvb_s2(crc, len & 0xFF);
    crc = crc8_dvb_s2(crc, (len >> 8) & 0xFF);

    for (uint16_t i = 0; i < len; i++) {
        out[idx++] = payload[i];
        crc = crc8_dvb_s2(crc, payload[i]);
    }
    out[idx++] = crc;

    msp_usb_write(out, idx);
    ble_msp_send(out, idx);
}

void msp_init(void) {
    state = MSP_STATE_IDLE;
    memset(&current_telem, 0, sizeof(current_telem));
    current_telem.cycle_time_us = 1000;
    for (int i = 0; i < 4; i++) current_telem.motor[i] = 1000;
    for (int i = 0; i < 14; i++) current_telem.rc_channels[i] = 1500;
    current_telem.rc_channels[2] = 1000; // Throttle low
}

void msp_update_telemetry(const msp_telemetry_t *telem) {
    if (telem) {
        memcpy(&current_telem, telem, sizeof(current_telem));
    }
}

static void msp_dispatch(uint16_t cmd, bool is_v2) {
    uint8_t buf[128];
    memset(buf, 0, sizeof(buf));

    switch (cmd) {
        case MSP_API_VERSION:
            buf[0] = 0;  // MSP_PROTOCOL_VERSION
            buf[1] = 1;  // API_VERSION_MAJOR
            buf[2] = 45; // API_VERSION_MINOR (Betaflight 4.5)
            if (is_v2) msp2_send_packet(cmd, buf, 3);
            else msp_send_packet((uint8_t)cmd, buf, 3);
            break;

        case MSP_FC_VARIANT:
            memcpy(buf, "BTFL", 4); // Betaflight
            if (is_v2) msp2_send_packet(cmd, buf, 4);
            else msp_send_packet((uint8_t)cmd, buf, 4);
            break;

        case MSP_FC_VERSION:
            buf[0] = 4; // Major
            buf[1] = 5; // Minor
            buf[2] = 0; // Patch
            if (is_v2) msp2_send_packet(cmd, buf, 3);
            else msp_send_packet((uint8_t)cmd, buf, 3);
            break;

        case MSP_BOARD_INFO: {
            // Target board identifier: "OSUX"
            memcpy(buf, "OSUX", 4);
            buf[4] = 0; buf[5] = 0; // Hardware revision
            buf[6] = 0;             // Target capabilities
            const char *board_name = "OsusX";
            uint8_t blen = strlen(board_name);
            buf[7] = blen;
            memcpy(&buf[8], board_name, blen);
            uint8_t pos = 8 + blen;
            buf[pos++] = blen;
            memcpy(&buf[pos], board_name, blen);
            pos += blen;
            buf[pos++] = 4;
            memcpy(&buf[pos], "OSUS", 4);
            pos += 4;
            if (is_v2) msp2_send_packet(cmd, buf, pos);
            else msp_send_packet((uint8_t)cmd, buf, pos);
            break;
        }

        case MSP_BUILD_INFO: {
            const char *build_str = "Oct 17 2026 12:00:00 (784d73a)";
            uint8_t len = strlen(build_str);
            if (is_v2) msp2_send_packet(cmd, (const uint8_t*)build_str, len);
            else msp_send_packet((uint8_t)cmd, (const uint8_t*)build_str, len);
            break;
        }

        case MSP_NAME: {
            const char *craft_name = "OsusX";
            uint8_t len = strlen(craft_name);
            if (is_v2) msp2_send_packet(cmd, (const uint8_t*)craft_name, len);
            else msp_send_packet((uint8_t)cmd, (const uint8_t*)craft_name, len);
            break;
        }

        case MSP_IDENT: {
            buf[0] = 45; // Version
            buf[1] = 3;  // Multitype: QuadX
            buf[2] = 0;  // MSP version
            buf[3] = 0; buf[4] = 0; buf[5] = 0; buf[6] = 0; // Capabilities
            if (is_v2) msp2_send_packet(cmd, buf, 7);
            else msp_send_packet((uint8_t)cmd, buf, 7);
            break;
        }

        case MSP_STATUS: {
            uint16_t cycle_time = current_telem.cycle_time_us > 0 ? current_telem.cycle_time_us : 1000;
            uint16_t i2c_errs = 0;
            // Betaflight sensor bits: Bit 0=ACC, Bit 1=BARO, Bit 2=MAG, Bit 3=GPS, Bit 4=RANGEFINDER, Bit 5=GYRO
            uint16_t sensor_flags = (1 << 0) | (1 << 1) | (1 << 4) | (1 << 5); // 0x33 (ACC + BARO + RANGE + GYRO)
            uint32_t flight_mode = current_telem.armed ? 0x01 : 0x00;

            buf[0] = cycle_time & 0xFF; buf[1] = cycle_time >> 8;
            buf[2] = i2c_errs & 0xFF;   buf[3] = i2c_errs >> 8;
            buf[4] = sensor_flags & 0xFF; buf[5] = sensor_flags >> 8;
            buf[6] = flight_mode & 0xFF; buf[7] = (flight_mode >> 8) & 0xFF;
            buf[8] = (flight_mode >> 16) & 0xFF; buf[9] = (flight_mode >> 24) & 0xFF;
            buf[10] = 0; // Config profile
            if (is_v2) msp2_send_packet(cmd, buf, 11);
            else msp_send_packet((uint8_t)cmd, buf, 11);
            break;
        }

        case MSP_STATUS_EX: {
            uint16_t cycle_time = current_telem.cycle_time_us > 0 ? current_telem.cycle_time_us : 1000;
            uint16_t i2c_errs = 0;
            uint16_t sensor_flags = (1 << 0) | (1 << 1) | (1 << 4) | (1 << 5); // ACC + BARO + RANGE + GYRO
            uint32_t flight_mode = current_telem.armed ? 0x01 : 0x00;
            uint8_t pid_profile = 0;
            uint16_t cpu_load = current_telem.cpu_load_pct;

            uint8_t p = 0;
            buf[p++] = cycle_time & 0xFF; buf[p++] = cycle_time >> 8;
            buf[p++] = i2c_errs & 0xFF;   buf[p++] = i2c_errs >> 8;
            buf[p++] = sensor_flags & 0xFF; buf[p++] = sensor_flags >> 8;
            buf[p++] = flight_mode & 0xFF; buf[p++] = (flight_mode >> 8) & 0xFF;
            buf[p++] = (flight_mode >> 16) & 0xFF; buf[p++] = (flight_mode >> 24) & 0xFF;
            buf[p++] = pid_profile;
            buf[p++] = cpu_load & 0xFF; buf[p++] = cpu_load >> 8;
            buf[p++] = 3; // PID_PROFILE_COUNT
            buf[p++] = 0; // currentControlRateProfileIndex
            buf[p++] = 0; // extra flightModeFlags byteCount
            buf[p++] = 32; // ARMING_DISABLE_FLAGS_COUNT
            buf[p++] = 0; buf[p++] = 0; buf[p++] = 0; buf[p++] = 0; // armingDisableFlags = 0 (Ready to ARM)
            buf[p++] = 0; // rebootRequired
            buf[p++] = 35; buf[p++] = 0; // coreTemp: 35C
            buf[p++] = 6; // CONTROL_RATE_PROFILE_COUNT
            buf[p++] = 3; // BATTERY_PROFILE_COUNT
            buf[p++] = 0; // currentBatteryProfileIndex

            if (is_v2) msp2_send_packet(cmd, buf, p);
            else msp_send_packet((uint8_t)cmd, buf, p);
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
            if (is_v2) msp2_send_packet(cmd, buf, 18);
            else msp_send_packet((uint8_t)cmd, buf, 18);
            break;

        case MSP_ATTITUDE: {
            buf[0] = current_telem.roll_decideg & 0xFF;
            buf[1] = current_telem.roll_decideg >> 8;
            buf[2] = current_telem.pitch_decideg & 0xFF;
            buf[3] = current_telem.pitch_decideg >> 8;
            int16_t yaw_deg = current_telem.yaw_decideg / 10;
            buf[4] = yaw_deg & 0xFF;
            buf[5] = yaw_deg >> 8;
            if (is_v2) msp2_send_packet(cmd, buf, 6);
            else msp_send_packet((uint8_t)cmd, buf, 6);
            break;
        }

        case MSP_ALTITUDE: {
            int32_t alt = current_telem.alt_cm;
            int16_t vario = current_telem.vario_cms;
            buf[0] = alt & 0xFF; buf[1] = (alt >> 8) & 0xFF;
            buf[2] = (alt >> 16) & 0xFF; buf[3] = (alt >> 24) & 0xFF;
            buf[4] = vario & 0xFF; buf[5] = vario >> 8;
            if (is_v2) msp2_send_packet(cmd, buf, 6);
            else msp_send_packet((uint8_t)cmd, buf, 6);
            break;
        }

        case MSP_ANALOG: {
            buf[0] = 111; // 11.1V (in 0.1V, for 3S)
            buf[1] = 0; buf[2] = 0; // mAh drawn
            buf[3] = 0xFF; buf[4] = 0x03; // RSSI: 1023
            buf[5] = 0; buf[6] = 0; // Current in 10mA
            if (is_v2) msp2_send_packet(cmd, buf, 7);
            else msp_send_packet((uint8_t)cmd, buf, 7);
            break;
        }

        case MSP_BATTERY_STATE: {
            buf[0] = 3;   // Cell count (3S)
            buf[1] = 450 & 0xFF; buf[2] = 450 >> 8; // Capacity: 450 mAh
            buf[3] = 111; // Voltage in 0.1V (11.1V)
            buf[4] = 0; buf[5] = 0; // mAh drawn
            buf[6] = 0; buf[7] = 0; // Amperage in 0.01A
            buf[8] = 0;   // Battery state: OK
            if (is_v2) msp2_send_packet(cmd, buf, 9);
            else msp_send_packet((uint8_t)cmd, buf, 9);
            break;
        }

        case MSP_MOTOR:
            for (int i = 0; i < 4; i++) {
                buf[i*2] = current_telem.motor[i] & 0xFF;
                buf[i*2 + 1] = current_telem.motor[i] >> 8;
            }
            if (is_v2) msp2_send_packet(cmd, buf, 8);
            else msp_send_packet((uint8_t)cmd, buf, 8);
            break;

        case MSP_MOTOR_CONFIG:
            buf[0] = 1070 & 0xFF; buf[1] = 1070 >> 8; // minthrottle
            buf[2] = 2000 & 0xFF; buf[3] = 2000 >> 8; // maxthrottle
            buf[4] = 1000 & 0xFF; buf[5] = 1000 >> 8; // mincommand
            buf[6] = 4; // Motor count
            buf[7] = 14; // Poles
            buf[8] = 0; // use_dshot_telemetry
            buf[9] = 0; // use_esc_sensor
            if (is_v2) msp2_send_packet(cmd, buf, 10);
            else msp_send_packet((uint8_t)cmd, buf, 10);
            break;

        case MSP_RC:
            for (int i = 0; i < 8; i++) {
                buf[i*2] = current_telem.rc_channels[i] & 0xFF;
                buf[i*2 + 1] = current_telem.rc_channels[i] >> 8;
            }
            if (is_v2) msp2_send_packet(cmd, buf, 16);
            else msp_send_packet((uint8_t)cmd, buf, 16);
            break;

        case MSP_PID:
            for (int i = 0; i < 3; i++) {
                buf[i*3] = pid_gains[i][0];
                buf[i*3 + 1] = pid_gains[i][1];
                buf[i*3 + 2] = pid_gains[i][2];
            }
            if (is_v2) msp2_send_packet(cmd, buf, 9);
            else msp_send_packet((uint8_t)cmd, buf, 9);
            break;

        case MSP_SET_PID:
            if (in_len >= 9) {
                for (int i = 0; i < 3; i++) {
                    pid_gains[i][0] = in_buf[i*3];
                    pid_gains[i][1] = in_buf[i*3 + 1];
                    pid_gains[i][2] = in_buf[i*3 + 2];
                }
            }
            if (is_v2) msp2_send_packet(cmd, NULL, 0);
            else msp_send_packet((uint8_t)cmd, NULL, 0);
            break;

        case MSP_BOXNAMES: {
            const char *boxes = "ARM;ANGLE;HORIZON;BARO;MAG;BEEPER;";
            uint8_t blen = strlen(boxes);
            if (is_v2) msp2_send_packet(cmd, (const uint8_t*)boxes, blen);
            else msp_send_packet((uint8_t)cmd, (const uint8_t*)boxes, blen);
            break;
        }

        case MSP_BOXIDS: {
            uint8_t boxids[] = { 0, 1, 2, 3, 4, 5 };
            if (is_v2) msp2_send_packet(cmd, boxids, sizeof(boxids));
            else msp_send_packet((uint8_t)cmd, boxids, sizeof(boxids));
            break;
        }

        case MSP_FEATURE_CONFIG: {
            uint32_t features = (1 << 0) | (1 << 3); // RX_SERIAL | MOTOR_STOP
            if (is_v2) msp2_send_packet(cmd, (uint8_t*)&features, 4);
            else msp_send_packet((uint8_t)cmd, (uint8_t*)&features, 4);
            break;
        }

        case MSP_UID: {
            uint8_t mac[6] = { 0xB0, 0x81, 0x84, 0x95, 0x4E, 0x68 };
            esp_read_mac(mac, ESP_MAC_WIFI_STA);
            uint32_t uid[3] = {
                (mac[0] << 24) | (mac[1] << 16) | (mac[2] << 8) | mac[3],
                (mac[4] << 8) | mac[5],
                0x55AA55AA
            };
            if (is_v2) msp2_send_packet(cmd, (uint8_t*)uid, 12);
            else msp_send_packet((uint8_t)cmd, (uint8_t*)uid, 12);
            break;
        }

        case MSP_SENSOR_CONFIG:
            buf[0] = 1; // ACC hardware: Bosch BMI270
            buf[1] = 0; // BARO hardware: none/default
            buf[2] = 0; // MAG hardware: none
            buf[3] = 0; // SONAR hardware: none
            if (is_v2) msp2_send_packet(cmd, buf, 4);
            else msp_send_packet((uint8_t)cmd, buf, 4);
            break;

        case MSP_SENSOR_ALIGNMENT:
            buf[0] = 0; // gyro alignment: CW0
            buf[1] = 0; // acc alignment: CW0
            buf[2] = 0; // mag alignment: CW0
            buf[3] = 0; // gyro detection flags
            buf[4] = 0; // gyro to use: primary (0)
            if (is_v2) msp2_send_packet(cmd, buf, 5);
            else msp_send_packet((uint8_t)cmd, buf, 5);
            break;

        case MSP_RC_TUNING:
            buf[0] = 100; // rcRate8
            buf[1] = 0;   // rcExpo8
            buf[2] = 70;  // rollRate
            buf[3] = 70;  // pitchRate
            buf[4] = 70;  // yawRate
            buf[5] = 0;   // dynThrPID
            buf[6] = 50;  // thrMid8
            buf[7] = 0;   // thrExpo8
            if (is_v2) msp2_send_packet(cmd, buf, 8);
            else msp_send_packet((uint8_t)cmd, buf, 8);
            break;

        case MSP_CF_SERIAL_CONFIG:
            // Report 1 serial port: VCP MSP at 115200
            buf[0] = 0; // port ID 0 = VCP
            buf[1] = 1; buf[2] = 0; // functionMask: 1 = MSP
            buf[3] = 5; // MSP baudrate: 115200
            buf[4] = 0; // GPS baud
            buf[5] = 0; // TELEM baud
            buf[6] = 0; // BLACKBOX baud
            if (is_v2) msp2_send_packet(cmd, buf, 7);
            else msp_send_packet((uint8_t)cmd, buf, 7);
            break;

        case MSP_ACC_TRIM:
            buf[0] = 0; buf[1] = 0; // pitch trim
            buf[2] = 0; buf[3] = 0; // roll trim
            if (is_v2) msp2_send_packet(cmd, buf, 4);
            else msp_send_packet((uint8_t)cmd, buf, 4);
            break;

        case MSP_MIXER_CONFIG:
            buf[0] = 3; // QuadX
            buf[1] = 0; // Normal rotation
            if (is_v2) msp2_send_packet(cmd, buf, 2);
            else msp_send_packet((uint8_t)cmd, buf, 2);
            break;

        case MSP_ADVANCED_CONFIG:
            buf[0] = 1; // gyro_sync_denom
            buf[1] = 1; // pid_process_denom
            buf[2] = 0; // use_unsyncedPwm
            buf[3] = 0; // fast_pwm_protocol: standard PWM
            buf[4] = 400 & 0xFF; buf[5] = 400 >> 8; // motor_pwm_rate: 400Hz
            buf[6] = 550 & 0xFF; buf[7] = 550 >> 8; // motorIdle: 5.5%
            buf[8] = 0; // gyroUse32Khz
            buf[9] = 0; // motorPwmInversion
            buf[10] = 0; // gyro_to_use
            buf[11] = 0; // gyroHighFsr
            buf[12] = 48; // gyroMovementCalibThreshold
            buf[13] = 125 & 0xFF; buf[14] = 125 >> 8; // gyroCalibDuration
            buf[15] = 0; buf[16] = 0; // gyroOffsetYaw
            buf[17] = 0; // gyroCheckOverflow
            if (is_v2) msp2_send_packet(cmd, buf, 18);
            else msp_send_packet((uint8_t)cmd, buf, 18);
            break;

        case MSP2_MCU_INFO: {
            const char *mcu_name = "ESP32-S3";
            buf[0] = 1;
            uint8_t nlen = strlen(mcu_name);
            memcpy(&buf[1], mcu_name, nlen);
            msp2_send_packet(cmd, buf, 1 + nlen);
            break;
        }

        case MSP_EEPROM_WRITE:
            if (is_v2) msp2_send_packet(cmd, NULL, 0);
            else msp_send_packet((uint8_t)cmd, NULL, 0);
            break;

        default:
            // Generic ACK for all other commands to prevent Betaflight Configurator from hanging
            if (is_v2) msp2_send_packet(cmd, NULL, 0);
            else msp_send_packet((uint8_t)cmd, NULL, 0);
            break;
    }
}

void msp_process_byte(uint8_t c) {
    switch (state) {
        case MSP_STATE_IDLE:
            if (c == '$') state = MSP_STATE_HEADER_DOLLAR;
            break;

        case MSP_STATE_HEADER_DOLLAR:
            if (c == 'M') state = MSP_STATE_HEADER_M;
            else if (c == 'X') state = MSP_STATE_HEADER_X;
            else state = MSP_STATE_IDLE;
            break;

        // ------------------ MSP V1 Handling ------------------
        case MSP_STATE_HEADER_M:
            state = (c == '<') ? MSP_STATE_V1_LEN : MSP_STATE_IDLE;
            break;

        case MSP_STATE_V1_LEN:
            in_len = c;
            in_checksum = c;
            in_idx = 0;
            state = MSP_STATE_V1_CMD;
            break;

        case MSP_STATE_V1_CMD:
            in_cmd = c;
            in_checksum ^= c;
            if (in_len > 0) {
                state = MSP_STATE_V1_PAYLOAD;
            } else {
                state = MSP_STATE_V1_CHECKSUM;
            }
            break;

        case MSP_STATE_V1_PAYLOAD:
            if (in_idx < sizeof(in_buf)) {
                in_buf[in_idx++] = c;
            }
            in_checksum ^= c;
            if (in_idx >= in_len) {
                state = MSP_STATE_V1_CHECKSUM;
            }
            break;

        case MSP_STATE_V1_CHECKSUM:
            if (c == in_checksum) {
                msp_dispatch(in_cmd, false);
            }
            state = MSP_STATE_IDLE;
            break;

        // ------------------ MSP V2 Handling ------------------
        case MSP_STATE_HEADER_X:
            state = (c == '<') ? MSP_STATE_V2_FLAG : MSP_STATE_IDLE;
            break;

        case MSP_STATE_V2_FLAG:
            in_flag = c;
            in_checksum = 0;
            in_checksum = crc8_dvb_s2(in_checksum, c);
            state = MSP_STATE_V2_CMD_L;
            break;

        case MSP_STATE_V2_CMD_L:
            in_cmd = c;
            in_checksum = crc8_dvb_s2(in_checksum, c);
            state = MSP_STATE_V2_CMD_H;
            break;

        case MSP_STATE_V2_CMD_H:
            in_cmd |= ((uint16_t)c << 8);
            in_checksum = crc8_dvb_s2(in_checksum, c);
            state = MSP_STATE_V2_LEN_L;
            break;

        case MSP_STATE_V2_LEN_L:
            in_len = c;
            in_checksum = crc8_dvb_s2(in_checksum, c);
            state = MSP_STATE_V2_LEN_H;
            break;

        case MSP_STATE_V2_LEN_H:
            in_len |= ((uint16_t)c << 8);
            in_checksum = crc8_dvb_s2(in_checksum, c);
            in_idx = 0;
            if (in_len > 0) {
                state = MSP_STATE_V2_PAYLOAD;
            } else {
                state = MSP_STATE_V2_CHECKSUM;
            }
            break;

        case MSP_STATE_V2_PAYLOAD:
            if (in_idx < sizeof(in_buf)) {
                in_buf[in_idx++] = c;
            }
            in_checksum = crc8_dvb_s2(in_checksum, c);
            if (in_idx >= in_len) {
                state = MSP_STATE_V2_CHECKSUM;
            }
            break;

        case MSP_STATE_V2_CHECKSUM:
            if (c == in_checksum) {
                msp_dispatch(in_cmd, true);
            }
            state = MSP_STATE_IDLE;
            break;
    }
}

void msp_usb_task_entry(void *pvParameters) {
    uint8_t rx_buf[64];
    while (1) {
        if (usb_serial_jtag_ll_rxfifo_data_available()) {
            int n = usb_serial_jtag_ll_read_rxfifo(rx_buf, sizeof(rx_buf));
            for (int i = 0; i < n; i++) {
                msp_process_byte(rx_buf[i]);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}
