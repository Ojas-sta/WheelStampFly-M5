#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t channels[14];  // 1000us - 2000us (Roll=0, Pitch=1, Thr=2, Yaw=3, AUX1-10=4..13)
    bool failsafe;
    uint32_t last_packet_time_ms;
} ibus_state_t;

void ibus_init(void);
bool ibus_update(void);
const ibus_state_t* ibus_get_state(void);

#ifdef __cplusplus
}
#endif
