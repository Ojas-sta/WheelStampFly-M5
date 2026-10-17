#include "bumper.h"
#include "../config.h"
#include "esp_log.h"

static const char *TAG = "BUMPER";
static bumper_status_t status = {
    .enabled = false,
    .front_dist_mm = 9999,
    .obstacle_warning = false,
    .active_braking = false
};

void bumper_init(void) {
    status.enabled = false;
    status.front_dist_mm = 9999;
    status.obstacle_warning = false;
    status.active_braking = false;
    ESP_LOGI(TAG, "Collision Avoidance Bumper initialized (Warn: %dmm, Brake: %dmm)", 
             BUMPER_WARN_DIST_MM, BUMPER_BRAKE_DIST_MM);
}

void bumper_set_enabled(bool enabled) {
    status.enabled = enabled;
}

void bumper_update_distance(uint16_t dist_mm) {
    status.front_dist_mm = dist_mm;

    if (!status.enabled) {
        status.obstacle_warning = false;
        status.active_braking = false;
        return;
    }

    status.obstacle_warning = (dist_mm < BUMPER_WARN_DIST_MM);
    status.active_braking = (dist_mm < BUMPER_BRAKE_DIST_MM);
}

float bumper_apply_pitch_limit(float pilot_pitch) {
    if (!status.enabled) {
        return pilot_pitch;
    }

    // Only intervene if pilot is pitching forward (pilot_pitch > 0)
    if (pilot_pitch <= 0.0f) {
        return pilot_pitch; // Allow pulling back freely at all times!
    }

    // Critical brake zone (< 500mm): clamp forward pitch to 0 and inject brief counter-tilt
    if (status.front_dist_mm < BUMPER_BRAKE_DIST_MM) {
        return -0.15f; // Soft reverse brake impulse to arrest inertia
    }

    // Proportional dampening zone (500mm - 1000mm)
    if (status.front_dist_mm < BUMPER_WARN_DIST_MM) {
        float scale = (float)(status.front_dist_mm - BUMPER_BRAKE_DIST_MM) / 
                      (float)(BUMPER_WARN_DIST_MM - BUMPER_BRAKE_DIST_MM);
        if (scale < 0.0f) scale = 0.0f;
        if (scale > 1.0f) scale = 1.0f;
        return pilot_pitch * scale;
    }

    return pilot_pitch;
}

const bumper_status_t* bumper_get_status(void) {
    return &status;
}
