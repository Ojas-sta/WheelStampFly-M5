#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void ble_msp_init(void);
void ble_msp_send(const uint8_t *data, size_t len);
bool ble_msp_is_connected(void);

#ifdef __cplusplus
}
#endif
