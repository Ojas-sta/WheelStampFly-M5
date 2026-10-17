#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float ax_g;     // Acceleration in g's
    float ay_g;
    float az_g;
    float gx_dps;   // Gyro rate in deg/sec
    float gy_dps;
    float gz_dps;
    float gx_rads;  // Gyro rate in rad/sec
    float gy_rads;
    float gz_rads;
} imu_data_t;

esp_err_t bmi270_init(void);
esp_err_t bmi270_read(imu_data_t *data);
void bmi270_calibrate(void);

#ifdef __cplusplus
}
#endif
