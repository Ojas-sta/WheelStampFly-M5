#include "front_tof.h"
#include "../config.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "FRONT_TOF";
#define VL53L3_DEFAULT_ADDR 0x29

esp_err_t front_tof_init(void) {
    // 1. Configure XSHUT pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FRONT_TOF_XSHUT_PIN) | (1ULL << BOTTOM_TOF_XSHUT_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // CRITICAL HARDWARE DECISION: Hold bottom ToF in permanent shutdown
    gpio_set_level(BOTTOM_TOF_XSHUT_PIN, 0); // Keep GPIO 7 LOW!

    // Keep front ToF in reset briefly, then enable
    gpio_set_level(FRONT_TOF_XSHUT_PIN, 0);
    esp_rom_delay_us(10000);
    gpio_set_level(FRONT_TOF_XSHUT_PIN, 1);
    esp_rom_delay_us(10000);

    // 2. Configure I2C bus
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    esp_err_t err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    ESP_LOGI(TAG, "I2C0 initialized. Bottom ToF held in sleep (GPIO 7=0). Front ToF enabled (GPIO 9=1)");
    return err;
}

bool front_tof_read_distance(uint16_t *dist_mm) {
    if (!dist_mm) return false;

    // Probe sensor communication register
    uint8_t reg = 0x01; // Model ID register
    uint8_t val = 0;
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, VL53L3_DEFAULT_ADDR, 
                                                 &reg, 1, &val, 1, pdMS_TO_TICKS(10));
    if (ret == ESP_OK) {
        // Mock valid distance reading until full ULD ranging loop starts
        *dist_mm = 1500; // 1.5m clear ahead
        return true;
    }
    return false;
}
