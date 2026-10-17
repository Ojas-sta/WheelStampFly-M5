#include "motor_pwm.h"
#include "../config.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "MOTOR_PWM";
static motor_protocol_t current_protocol = MOTOR_PROTO_PWM400;

static const ledc_channel_t CHANNELS[4] = {
    LEDC_CHANNEL_0, // FL (GPIO 5)
    LEDC_CHANNEL_1, // FR (GPIO 42)
    LEDC_CHANNEL_2, // RL (GPIO 10)
    LEDC_CHANNEL_3  // RR (GPIO 41)
};

static const gpio_num_t MOTOR_PINS[4] = {
    MOTOR_FL_PIN,
    MOTOR_FR_PIN,
    MOTOR_RL_PIN,
    MOTOR_RR_PIN
};

void motor_pwm_init(motor_protocol_t protocol) {
    current_protocol = protocol;

    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = MOTOR_PWM_RESOLUTION,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = MOTOR_PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch_conf = {
            .gpio_num   = MOTOR_PINS[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = CHANNELS[i],
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = MOTOR_PWM_MIN_TICKS, // Initialize safely at 1000us neutral
            .hpoint     = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));
    }

    ESP_LOGI(TAG, "Initialized 4x 400Hz Brushless ESC PWM channels (FL:5, FR:42, RL:10, RR:41)");
    motor_pwm_disarm();
}

void motor_pwm_write(float fl, float fr, float rl, float rr) {
    float throttles[4] = { fl, fr, rl, rr };

    for (int i = 0; i < 4; i++) {
        // Clamp strictly between 0.0f and 1.0f
        float val = throttles[i];
        if (val < 0.0f) val = 0.0f;
        if (val > 1.0f) val = 1.0f;

        uint32_t duty_ticks = (uint32_t)(MOTOR_PWM_MIN_TICKS + (val * MOTOR_PWM_RANGE_TICKS));
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CHANNELS[i], duty_ticks);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, CHANNELS[i]);
    }
}

void motor_pwm_disarm(void) {
    for (int i = 0; i < 4; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CHANNELS[i], MOTOR_PWM_MIN_TICKS);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, CHANNELS[i]);
    }
}

void motor_pwm_set_protocol(motor_protocol_t protocol) {
    current_protocol = protocol;
    motor_pwm_init(protocol);
}
