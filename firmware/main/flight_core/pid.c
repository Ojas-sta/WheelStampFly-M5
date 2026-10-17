#include "pid.h"

static pid_system_t pids;

void pid_init(void) {
    // Outer Angle Loop (Maps angle error in deg to target rate in deg/s)
    pids.angle_roll.kp  = 4.5f;
    pids.angle_pitch.kp = 4.5f;

    // Inner Rate Loop (Tuned for 1104 Brushless Motors on 3S LiPo)
    pids.rate_roll.kp = 0.38f;
    pids.rate_roll.ki = 0.30f;
    pids.rate_roll.kd = 0.023f;
    pids.rate_roll.max_integral = 0.20f;
    pids.rate_roll.max_output = 0.40f;

    pids.rate_pitch.kp = 0.42f;
    pids.rate_pitch.ki = 0.33f;
    pids.rate_pitch.kd = 0.026f;
    pids.rate_pitch.max_integral = 0.20f;
    pids.rate_pitch.max_output = 0.40f;

    pids.rate_yaw.kp = 0.75f;
    pids.rate_yaw.ki = 0.45f;
    pids.rate_yaw.kd = 0.000f; // Minimal D on yaw
    pids.rate_yaw.max_integral = 0.25f;
    pids.rate_yaw.max_output = 0.40f;

    pid_reset_integrals();
}

void pid_reset_integrals(void) {
    pids.rate_roll.integral = 0.0f;
    pids.rate_roll.prev_error = 0.0f;
    pids.rate_pitch.integral = 0.0f;
    pids.rate_pitch.prev_error = 0.0f;
    pids.rate_yaw.integral = 0.0f;
    pids.rate_yaw.prev_error = 0.0f;
}

static float pid_compute_axis(pid_axis_t *axis, float target, float actual, float dt) {
    float error = target - actual;

    // Proportional
    float p = axis->kp * error;

    // Integral with anti-windup clamping
    axis->integral += axis->ki * error * dt;
    if (axis->integral > axis->max_integral) axis->integral = axis->max_integral;
    if (axis->integral < -axis->max_integral) axis->integral = -axis->max_integral;

    // Derivative on error
    float d = 0.0f;
    if (dt > 0.0001f) {
        d = axis->kd * (error - axis->prev_error) / dt;
    }
    axis->prev_error = error;

    float output = p + axis->integral + d;
    if (output > axis->max_output) output = axis->max_output;
    if (output < -axis->max_output) output = -axis->max_output;

    return output;
}

void pid_update_angle(float target_roll_deg, float target_pitch_deg,
                      float actual_roll_deg, float actual_pitch_deg,
                      float *out_roll_rate, float *out_pitch_rate) {
    if (out_roll_rate) {
        *out_roll_rate = pids.angle_roll.kp * (target_roll_deg - actual_roll_deg);
    }
    if (out_pitch_rate) {
        *out_pitch_rate = pids.angle_pitch.kp * (target_pitch_deg - actual_pitch_deg);
    }
}

void pid_update_rates(float target_roll_rate, float target_pitch_rate, float target_yaw_rate,
                      float actual_roll_rate, float actual_pitch_rate, float actual_yaw_rate,
                      float dt, float *out_roll, float *out_pitch, float *out_yaw) {
    if (out_roll) {
        *out_roll = pid_compute_axis(&pids.rate_roll, target_roll_rate, actual_roll_rate, dt);
    }
    if (out_pitch) {
        *out_pitch = pid_compute_axis(&pids.rate_pitch, target_pitch_rate, actual_pitch_rate, dt);
    }
    if (out_yaw) {
        *out_yaw = pid_compute_axis(&pids.rate_yaw, target_yaw_rate, actual_yaw_rate, dt);
    }
}
