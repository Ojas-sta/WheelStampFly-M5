#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t front_tof_init(void);
bool front_tof_read_distance(uint16_t *dist_mm);

#ifdef __cplusplus
}
#endif
