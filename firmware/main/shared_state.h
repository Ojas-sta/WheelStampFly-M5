#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "flight_core/ahrs.h"
#include "comms/ibus.h"
#include "sensors/stp23l.h"
#include "safety/bumper.h"
#include "safety/ui_led_buzzer.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // RC State (Written by Core 0, Read by Core 1)
    uint16_t rc_channels[14];
    bool rc_failsafe;

    // Flight Dynamics (Written by Core 1, Read by Core 0)
    ahrs_t attitude;
    int16_t acc_raw[3];    // Live IMU Accelerometer (X, Y, Z)
    int16_t gyro_raw[3];   // Live IMU Gyroscope (X, Y, Z)
    float motor_duties[4]; // FL, FR, RL, RR (0.0 to 1.0)
    bool armed;
    flight_state_t flight_state;

    // High Precision Performance Timing
    uint32_t loop_dt_us;    // Actual cycle period in microseconds (target 1000us)
    uint32_t exec_us;       // Actual calculation time per cycle (target < 50us)
    uint8_t cpu_load_pct;   // Core 1 CPU load (0-100%)

    // Environmental Ranging (Written by Core 0, Read by Core 1)
    uint16_t front_dist_mm;
    uint16_t down_dist_mm;
    bool bumper_active;
    bool alt_hold_active;
} drone_shared_state_t;

extern drone_shared_state_t g_drone;

#ifdef __cplusplus
}
#endif
