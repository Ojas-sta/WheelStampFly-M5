#include "ibus.h"
#include "../config.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "IBUS";
static ibus_state_t state;
static uint8_t rx_buffer[64];
static uint8_t packet[32];
static uint8_t packet_idx = 0;

void ibus_init(void) {
    uart_config_t uart_config = {
        .baud_rate  = RC_IBUS_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(RC_UART_NUM, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RC_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(RC_UART_NUM, RC_TX_PIN, RC_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    memset(&state, 0, sizeof(state));
    for (int i = 0; i < 14; i++) {
        state.channels[i] = 1500;
    }
    state.channels[2] = 1000; // Throttle down by default
    state.failsafe = true;

    ESP_LOGI(TAG, "FlySky i-Bus initialized on Port J2 (UART1 RX: GPIO %d)", RC_RX_PIN);
}

bool ibus_update(void) {
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    int len = uart_read_bytes(RC_UART_NUM, rx_buffer, sizeof(rx_buffer), 0);
    bool new_frame = false;

    for (int i = 0; i < len; i++) {
        uint8_t byte = rx_buffer[i];

        if (packet_idx == 0) {
            if (byte == 0x20) {
                packet[packet_idx++] = byte;
            }
        } else if (packet_idx == 1) {
            if (byte == 0x40) {
                packet[packet_idx++] = byte;
            } else {
                packet_idx = 0;
            }
        } else {
            packet[packet_idx++] = byte;

            if (packet_idx == 32) {
                // Verify 16-bit checksum
                uint16_t checksum = 0xFFFF;
                for (int j = 0; j < 30; j++) {
                    checksum -= packet[j];
                }
                uint16_t packet_checksum = packet[30] | (packet[31] << 8);

                if (checksum == packet_checksum) {
                    for (int ch = 0; ch < 14; ch++) {
                        uint16_t val = packet[2 + (ch * 2)] | (packet[3 + (ch * 2)] << 8);
                        if (val >= 800 && val <= 2200) {
                            state.channels[ch] = val;
                        }
                    }
                    state.last_packet_time_ms = now_ms;
                    state.failsafe = false;
                    new_frame = true;
                }
                packet_idx = 0;
            }
        }
    }

    // Failsafe watchdog check
    if (now_ms - state.last_packet_time_ms > RC_FAILSAFE_TIMEOUT_MS) {
        state.failsafe = true;
        state.channels[2] = 1000; // Force throttle to zero on failsafe
    }

    return new_frame;
}

const ibus_state_t* ibus_get_state(void) {
    return &state;
}
