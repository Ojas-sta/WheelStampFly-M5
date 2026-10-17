#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool enabled;            // Toggled via AUX switch
    uint16_t front_dist_mm;  // Reading from front VL53L3
    bool obstacle_warning;   // True if < 1.0m
    bool active_braking;     // True if < 0.5m
} bumper_status_t;

void bumper_init(void);
void bumper_set_enabled(bool enabled);
void bumper_update_distance(uint16_t dist_mm);
float bumper_apply_pitch_limit(float pilot_pitch_input); // Input -1.0f (pull back) to +1.0f (push forward)
const bumper_status_t* bumper_get_status(void);

#ifdef __cplusplus
}
#endif
