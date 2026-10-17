#include "stp23l.h"
#include "../config.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "STP23L";
static stp23l_data_t data;
static uint8_t rx_buffer[128];
static uint8_t packet[9];
static uint8_t packet_idx = 0;

void stp23l_init(void) {
    uart_config_t uart_config = {
        .baud_rate  = LIDAR_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LIDAR_UART_NUM, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LIDAR_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART_NUM, LIDAR_TX_PIN, LIDAR_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    memset(&data, 0, sizeof(data));
    ESP_LOGI(TAG, "STP23L Downward 1D LiDAR initialized on Port J3 (UART2 RX: GPIO %d)", LIDAR_RX_PIN);
}

bool stp23l_update(void) {
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    int len = uart_read_bytes(LIDAR_UART_NUM, rx_buffer, sizeof(rx_buffer), 0);
    bool new_data = false;

    // Standard single-point LiDAR framing (0x59 0x59 format)
    for (int i = 0; i < len; i++) {
        uint8_t b = rx_buffer[i];

        if (packet_idx == 0) {
            if (b == 0x59) packet[packet_idx++] = b;
        } else if (packet_idx == 1) {
            if (b == 0x59) packet[packet_idx++] = b;
            else packet_idx = 0;
        } else {
            packet[packet_idx++] = b;
            if (packet_idx == 9) {
                uint8_t checksum = 0;
                for (int j = 0; j < 8; j++) checksum += packet[j];

                if (checksum == packet[8]) {
                    data.distance_mm = packet[2] | (packet[3] << 8);
                    data.signal_quality = packet[4] | (packet[5] << 8);
                    data.valid = (data.distance_mm > 20 && data.distance_mm < 8000);
                    data.last_update_ms = now_ms;
                    new_data = true;
                }
                packet_idx = 0;
            }
        }
    }

    if (now_ms - data.last_update_ms > 250) {
        data.valid = false;
    }

    return new_data;
}

const stp23l_data_t* stp23l_get_data(void) {
    return &data;
}
