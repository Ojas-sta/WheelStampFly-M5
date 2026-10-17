#include "ahrs.h"
#include <math.h>

#define TWO_KP (2.0f * 0.5f) // 2 * proportional gain
#define TWO_KI (2.0f * 0.0f) // 2 * integral gain

static ahrs_t ahrs = {
    .roll_deg = 0.0f,
    .pitch_deg = 0.0f,
    .yaw_deg = 0.0f,
    .q = { 1.0f, 0.0f, 0.0f, 0.0f }
};

static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

void ahrs_init(float sample_rate_hz) {
    ahrs.q[0] = 1.0f;
    ahrs.q[1] = 0.0f;
    ahrs.q[2] = 0.0f;
    ahrs.q[3] = 0.0f;
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
}

void ahrs_update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Normalise accelerometer measurement
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity
        halfvx = ahrs.q[1] * ahrs.q[3] - ahrs.q[0] * ahrs.q[2];
        halfvy = ahrs.q[0] * ahrs.q[1] + ahrs.q[2] * ahrs.q[3];
        halfvz = ahrs.q[0] * ahrs.q[0] - 0.5f + ahrs.q[3] * ahrs.q[3];

        // Error is sum of cross product between estimated direction and measured direction of field
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Apply feedback only if valid
        if (TWO_KI > 0.0f) {
            integralFBx += TWO_KI * halfex * dt;
            integralFBy += TWO_KI * halfey * dt;
            integralFBz += TWO_KI * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }

        gx += TWO_KP * halfex;
        gy += TWO_KP * halfey;
        gz += TWO_KP * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = ahrs.q[0];
    qb = ahrs.q[1];
    qc = ahrs.q[2];
    ahrs.q[0] += (-qb * gx - qc * gy - ahrs.q[3] * gz);
    ahrs.q[1] += (qa * gx + qc * gz - ahrs.q[3] * gy);
    ahrs.q[2] += (qa * gy - qb * gz + ahrs.q[3] * gx);
    ahrs.q[3] += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = 1.0f / sqrtf(ahrs.q[0] * ahrs.q[0] + ahrs.q[1] * ahrs.q[1] + 
                             ahrs.q[2] * ahrs.q[2] + ahrs.q[3] * ahrs.q[3]);
    ahrs.q[0] *= recipNorm;
    ahrs.q[1] *= recipNorm;
    ahrs.q[2] *= recipNorm;
    ahrs.q[3] *= recipNorm;

    // Convert to Euler angles (degrees)
    ahrs.roll_deg = atan2f(2.0f * (ahrs.q[0] * ahrs.q[1] + ahrs.q[2] * ahrs.q[3]),
                           1.0f - 2.0f * (ahrs.q[1] * ahrs.q[1] + ahrs.q[2] * ahrs.q[2])) * (180.0f / M_PI);
    ahrs.pitch_deg = asinf(2.0f * (ahrs.q[0] * ahrs.q[2] - ahrs.q[3] * ahrs.q[1])) * (180.0f / M_PI);
    ahrs.yaw_deg = atan2f(2.0f * (ahrs.q[0] * ahrs.q[3] + ahrs.q[1] * ahrs.q[2]),
                          1.0f - 2.0f * (ahrs.q[2] * ahrs.q[2] + ahrs.q[3] * ahrs.q[3])) * (180.0f / M_PI);
}

const ahrs_t* ahrs_get_attitude(void) {
    return &ahrs;
}
