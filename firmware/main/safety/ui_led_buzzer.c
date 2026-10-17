#include "ui_led_buzzer.h"
#include "../config.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "UI";
static uint32_t last_cycle_ms = 0;
static bool buzzer_state = false;

void ui_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUZZER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(BUZZER_GPIO, 0);

    ESP_LOGI(TAG, "UI initialized (Buzzer on GPIO %d, RGB LED on GPIO %d)", BUZZER_GPIO, WS2812_RGB_GPIO);
}

void ui_update(flight_state_t state, bool obstacle_warning, bool active_braking) {
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    // Audio/Visual state machine running on Core 0
    if (active_braking) {
        // Continuous high-pitched chirp when critical obstacle detected
        if (now_ms - last_cycle_ms > 80) {
            buzzer_state = !buzzer_state;
            gpio_set_level(BUZZER_GPIO, buzzer_state ? 1 : 0);
            last_cycle_ms = now_ms;
        }
    } else if (obstacle_warning) {
        // Intermittent double-chirp when in warning bubble
        if (now_ms - last_cycle_ms > 200) {
            buzzer_state = !buzzer_state;
            gpio_set_level(BUZZER_GPIO, buzzer_state ? 1 : 0);
            last_cycle_ms = now_ms;
        }
    } else if (state == FLIGHT_STATE_FAILSAFE) {
        // Continuous alarm on failsafe
        if (now_ms - last_cycle_ms > 150) {
            buzzer_state = !buzzer_state;
            gpio_set_level(BUZZER_GPIO, buzzer_state ? 1 : 0);
            last_cycle_ms = now_ms;
        }
    } else {
        gpio_set_level(BUZZER_GPIO, 0);
    }
}
