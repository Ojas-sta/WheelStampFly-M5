#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // Attitude for Betaflight 3D Model in Setup tab
    int16_t roll_decideg;   // Roll in 0.1 deg (-1800 to 1800)
    int16_t pitch_decideg;  // Pitch in 0.1 deg (-1800 to 1800)
    int16_t yaw_decideg;    // Yaw in 0.1 deg (-1800 to 1800)

    // Raw IMU
    int16_t acc[3];
    int16_t gyro[3];
    int16_t mag[3];

    // Altitude
    int32_t alt_cm;
    int16_t vario_cms;

    // Battery
    uint16_t vbat_mv;
    uint16_t current_ma;

    // Motors
    uint16_t motor[4];     // 1000 - 2000

    // RC Channels
    uint16_t rc_channels[14]; // 1000 - 2000

    // Flight Status & Real Timing
    bool armed;
    uint32_t flight_mode_flags;
    uint16_t cycle_time_us; // Actual loop cycle time in microseconds
    uint8_t cpu_load_pct;   // Actual CPU usage (0-100%)
} msp_telemetry_t;

void msp_init(void);
void msp_process_byte(uint8_t c);
void msp_update_telemetry(const msp_telemetry_t *telem);
void msp_usb_task_entry(void *pvParameters);

#ifdef __cplusplus
}
#endif
