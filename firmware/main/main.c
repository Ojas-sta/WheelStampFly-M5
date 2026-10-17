#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_pm.h"
#include "soc/rtc.h"

#include "config.h"
#include "shared_state.h"
#include "flight_core/motor_pwm.h"
#include "flight_core/ahrs.h"
#include "flight_core/pid.h"
#include "sensors/bmi270.h"
#include "sensors/front_tof.h"
#include "sensors/stp23l.h"
#include "comms/ibus.h"
#include "comms/msp.h"
#include "comms/ble_msp.h"
#include "safety/bumper.h"
#include "safety/ui_led_buzzer.h"

static const char *TAG = "OsusX";
drone_shared_state_t g_drone;

// ==============================================================================
// CORE 1: HARD REAL-TIME FLIGHT CONTROL TASK (1000 Hz / 1ms dt)
// ==============================================================================
static void flight_core_task(void *pvParameters) {
    ESP_LOGI(TAG, "Flight Core Task running on Core %d @ 1000Hz", xPortGetCoreID());

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1); // 1ms
    const float dt = 0.001f;

    imu_data_t imu;
    int64_t last_cycle_us = esp_timer_get_time();

    while (1) {
        vTaskDelayUntil(&last_wake_time, period);

        int64_t t_start = esp_timer_get_time();
        int64_t dt_actual_us = t_start - last_cycle_us;
        last_cycle_us = t_start;

        // 1. Read IMU (BMI270 over SPI @ 10MHz)
        if (bmi270_read(&imu) == ESP_OK) {
            // 2. Update Mahony AHRS Attitude Filter
            ahrs_update(imu.gx_rads, imu.gy_rads, imu.gz_rads,
                        imu.ax_g, imu.ay_g, imu.az_g, dt);
            g_drone.attitude = *ahrs_get_attitude();

            // Store raw IMU values for Betaflight Telemetry
            g_drone.acc_raw[0] = (int16_t)(imu.ax_g * 512.0f);
            g_drone.acc_raw[1] = (int16_t)(imu.ay_g * 512.0f);
            g_drone.acc_raw[2] = (int16_t)(imu.az_g * 512.0f);

            g_drone.gyro_raw[0] = (int16_t)(imu.gx_dps * 16.4f);
            g_drone.gyro_raw[1] = (int16_t)(imu.gy_dps * 16.4f);
            g_drone.gyro_raw[2] = (int16_t)(imu.gz_dps * 16.4f);
        }

        // 3. Check RC Arming & Failsafe
        bool rc_failsafe = g_drone.rc_failsafe;
        bool arm_switch = (g_drone.rc_channels[4] > 1700); // AUX1
        uint16_t throttle_raw = g_drone.rc_channels[2];    // CH3 (1000 - 2000)

        if (arm_switch && !rc_failsafe && throttle_raw > 1050) {
            g_drone.armed = true;
            g_drone.flight_state = FLIGHT_STATE_ARMED;
        } else {
            g_drone.armed = false;
            g_drone.flight_state = rc_failsafe ? FLIGHT_STATE_FAILSAFE : FLIGHT_STATE_DISARMED;
        }

        if (g_drone.armed) {
            // Normalize pilot inputs (-1.0f to +1.0f)
            float pilot_roll  = (float)((int16_t)g_drone.rc_channels[0] - 1500) / 500.0f;
            float pilot_pitch = (float)((int16_t)g_drone.rc_channels[1] - 1500) / 500.0f;
            float pilot_thr   = (float)(throttle_raw - 1000) / 1000.0f;
            float pilot_yaw   = (float)((int16_t)g_drone.rc_channels[3] - 1500) / 500.0f;

            // 4. Apply Collision Bumper Protection on Forward Pitch
            if (g_drone.bumper_active) {
                pilot_pitch = bumper_apply_pitch_limit(pilot_pitch);
            }

            // 5. Outer Angle Loop (Max +/- 35 deg angle)
            float target_roll_deg  = pilot_roll * 35.0f;
            float target_pitch_deg = pilot_pitch * 35.0f;
            float target_roll_rate, target_pitch_rate;

            pid_update_angle(target_roll_deg, target_pitch_deg,
                             g_drone.attitude.roll_deg, g_drone.attitude.pitch_deg,
                             &target_roll_rate, &target_pitch_rate);

            float target_yaw_rate = pilot_yaw * 200.0f; // Max 200 deg/s yaw

            // 6. Inner Rate Loop (Torque outputs)
            float corr_roll, corr_pitch, corr_yaw;
            pid_update_rates(target_roll_rate, target_pitch_rate, target_yaw_rate,
                             imu.gx_dps, imu.gy_dps, imu.gz_dps,
                             dt, &corr_roll, &corr_pitch, &corr_yaw);

            // 7. Quad-X Brushless Motor Mixing
            // FL (CW), FR (CCW), RL (CCW), RR (CW)
            float fl = pilot_thr + corr_pitch + corr_roll - corr_yaw;
            float fr = pilot_thr + corr_pitch - corr_roll + corr_yaw;
            float rl = pilot_thr - corr_pitch + corr_roll + corr_yaw;
            float rr = pilot_thr - corr_pitch - corr_roll - corr_yaw;

            motor_pwm_write(fl, fr, rl, rr);

            g_drone.motor_duties[0] = fl;
            g_drone.motor_duties[1] = fr;
            g_drone.motor_duties[2] = rl;
            g_drone.motor_duties[3] = rr;
        } else {
            // Safe Disarmed State
            motor_pwm_disarm();
            pid_reset_integrals();
            g_drone.motor_duties[0] = 0.0f;
            g_drone.motor_duties[1] = 0.0f;
            g_drone.motor_duties[2] = 0.0f;
            g_drone.motor_duties[3] = 0.0f;
        }

        int64_t t_end = esp_timer_get_time();
        int64_t exec_us = t_end - t_start;

        g_drone.loop_dt_us = (uint32_t)dt_actual_us;
        g_drone.exec_us = (uint32_t)exec_us;
        if (dt_actual_us > 0) {
            g_drone.cpu_load_pct = (uint8_t)((exec_us * 100) / dt_actual_us);
        }
    }
}

// ==============================================================================
// CORE 0: ASYNCHRONOUS SERVICE & TELEMETRY TASK (100 Hz / 10ms dt)
// ==============================================================================
static void service_task(void *pvParameters) {
    ESP_LOGI(TAG, "Service Task running on Core %d @ 100Hz", xPortGetCoreID());

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10); // 10ms

    msp_telemetry_t telem;
    uint32_t log_counter = 0;

    while (1) {
        vTaskDelayUntil(&last_wake_time, period);

        // 1. Process FlySky i-Bus Radio Input
        if (ibus_update()) {
            const ibus_state_t *rc = ibus_get_state();
            for (int i = 0; i < 14; i++) {
                g_drone.rc_channels[i] = rc->channels[i];
            }
            g_drone.rc_failsafe = rc->failsafe;
        } else {
            const ibus_state_t *rc = ibus_get_state();
            g_drone.rc_failsafe = rc->failsafe;
        }

        // 2. Process Downward STP23L LiDAR Input
        if (stp23l_update()) {
            const stp23l_data_t *lidar = stp23l_get_data();
            if (lidar->valid) {
                g_drone.down_dist_mm = lidar->distance_mm;
            }
        }

        // 3. Process Front VL53L3 ToF Bumper
        uint16_t front_dist = 0;
        if (front_tof_read_distance(&front_dist)) {
            g_drone.front_dist_mm = front_dist;
            bumper_update_distance(front_dist);
        }

        // 4. Update AUX Mode Switches
        // AUX2 (CH6) > 1500 = Bumper Active
        g_drone.bumper_active = (g_drone.rc_channels[5] > 1500);
        bumper_set_enabled(g_drone.bumper_active);

        // AUX3 (CH7) > 1500 = AltHold Active
        g_drone.alt_hold_active = (g_drone.rc_channels[6] > 1500);

        // 5. Update Audio/Visual UI (WS2812 & Buzzer) on Core 0
        const bumper_status_t *b_status = bumper_get_status();
        ui_update(g_drone.flight_state, b_status->obstacle_warning, b_status->active_braking);

        // 6. Process Betaflight Configurator MSP Telemetry
        telem.roll_decideg  = (int16_t)(g_drone.attitude.roll_deg * 10.0f);
        telem.pitch_decideg = (int16_t)(g_drone.attitude.pitch_deg * 10.0f);
        telem.yaw_decideg   = (int16_t)(g_drone.attitude.yaw_deg * 10.0f);
        telem.alt_cm        = g_drone.down_dist_mm / 10;
        telem.armed         = g_drone.armed;
        telem.cycle_time_us = (uint16_t)g_drone.loop_dt_us;
        telem.cpu_load_pct  = g_drone.cpu_load_pct;

        for (int i = 0; i < 3; i++) {
            telem.acc[i] = g_drone.acc_raw[i];
            telem.gyro[i] = g_drone.gyro_raw[i];
            telem.mag[i] = 0;
        }

        for (int i = 0; i < 4; i++) {
            telem.motor[i] = 1000 + (uint16_t)(g_drone.motor_duties[i] * 1000.0f);
        }
        for (int i = 0; i < 14; i++) {
            telem.rc_channels[i] = g_drone.rc_channels[i];
        }

        msp_update_telemetry(&telem);
    }
}

// ==============================================================================
// ENTRY POINT
// ==============================================================================
void app_main(void) {
    // Initialize NVS (required for non-volatile config storage and BLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    memset(&g_drone, 0, sizeof(g_drone));
    g_drone.loop_dt_us = 1000;
    for (int i = 0; i < 14; i++) g_drone.rc_channels[i] = 1500;
    g_drone.rc_channels[2] = 1000; // Throttle
    g_drone.rc_failsafe = true;

    // Initialize Subsystems
    motor_pwm_init(MOTOR_PROTO_PWM400);
    ahrs_init(1000.0f);
    pid_init();
    bmi270_init();
    front_tof_init();
    stp23l_init();
    ibus_init();
    msp_init();
    bumper_init();
    ui_init();

    // Initialize Bluetooth BLE Betaflight Server on Core 0
    ble_msp_init();

    // Spawn Flight Control Task strictly on Core 1 (Highest Priority, 1000Hz)
    xTaskCreatePinnedToCore(flight_core_task, "flight_core", 4096, NULL, 
                            FLIGHT_TASK_PRIORITY, NULL, FLIGHT_TASK_CORE);

    // Spawn Service/Telemetry Task strictly on Core 0 (Background, 100Hz)
    xTaskCreatePinnedToCore(service_task, "service_core", 4096, NULL, 
                            SERVICE_TASK_PRIORITY, NULL, SERVICE_TASK_CORE);

    // Spawn USB-MSP Event-driven Receiver Task strictly on Core 0
    xTaskCreatePinnedToCore(msp_usb_task_entry, "msp_usb", 4096, NULL, 
                            5, NULL, 0);

    ESP_LOGI(TAG, "BOOT COMPLETE: Tasks created successfully");
}


