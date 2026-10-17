#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float q[4]; // Quaternion [w, x, y, z]
} ahrs_t;

void ahrs_init(float sample_rate_hz);
void ahrs_update(float gx_rads, float gy_rads, float gz_rads, 
                 float ax_g, float ay_g, float az_g, float dt_sec);
const ahrs_t* ahrs_get_attitude(void);

#ifdef __cplusplus
}
#endif
