#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_PROTO_PWM400 = 0,
    MOTOR_PROTO_DSHOT300 = 1
} motor_protocol_t;

void motor_pwm_init(motor_protocol_t protocol);
void motor_pwm_write(float fl, float fr, float rl, float rr); // Inputs 0.0f (idle) to 1.0f (max)
void motor_pwm_disarm(void);
void motor_pwm_set_protocol(motor_protocol_t protocol);

#ifdef __cplusplus
}
#endif
