#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float max_integral;
    float max_output;
} pid_axis_t;

typedef struct {
    pid_axis_t rate_roll;
    pid_axis_t rate_pitch;
    pid_axis_t rate_yaw;
    pid_axis_t angle_roll;
    pid_axis_t angle_pitch;
} pid_system_t;

void pid_init(void);
void pid_reset_integrals(void);
void pid_update_rates(float target_roll_rate, float target_pitch_rate, float target_yaw_rate,
                      float actual_roll_rate, float actual_pitch_rate, float actual_yaw_rate,
                      float dt, float *out_roll, float *out_pitch, float *out_yaw);
void pid_update_angle(float target_roll_deg, float target_pitch_deg,
                      float actual_roll_deg, float actual_pitch_deg,
                      float *out_roll_rate, float *out_pitch_rate);

#ifdef __cplusplus
}
#endif
