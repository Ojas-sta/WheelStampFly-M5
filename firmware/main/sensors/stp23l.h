#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t distance_mm;
    uint8_t signal_quality;
    bool valid;
    uint32_t last_update_ms;
} stp23l_data_t;

void stp23l_init(void);
bool stp23l_update(void);
const stp23l_data_t* stp23l_get_data(void);

#ifdef __cplusplus
}
#endif
