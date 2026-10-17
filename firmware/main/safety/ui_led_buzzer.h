#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FLIGHT_STATE_DISARMED = 0,
    FLIGHT_STATE_ARMED = 1,
    FLIGHT_STATE_ASSIST_HOVER = 2,
    FLIGHT_STATE_FAILSAFE = 3
} flight_state_t;

void ui_init(void);
void ui_update(flight_state_t state, bool obstacle_warning, bool active_braking);

#ifdef __cplusplus
}
#endif
