#include "bmi270.h"
#include "bmi270_config.h"
#include "../config.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <math.h>

static const char *TAG = "BMI270";
static spi_device_handle_t spi_dev = NULL;

// Registers
#define BMI270_REG_CHIP_ID          0x00
#define BMI270_REG_ERR_REG          0x02
#define BMI270_REG_STATUS           0x03
#define BMI270_REG_DATA             0x0C
#define BMI270_REG_INTERNAL_STATUS  0x21
#define BMI270_REG_ACC_CONF         0x40
#define BMI270_REG_ACC_RANGE        0x41
#define BMI270_REG_GYR_CONF         0x42
#define BMI270_REG_GYR_RANGE        0x43
#define BMI270_REG_INIT_CTRL        0x59
#define BMI270_REG_INIT_DATA        0x5E
#define BMI270_REG_PWR_CONF         0x7C
#define BMI270_REG_PWR_CTRL         0x7D
#define BMI270_REG_CMD              0x7E

#define BMI270_CMD_SOFTRESET        0xB6

// DMA-safe memory buffers
static uint8_t *s_tx_buf = NULL;
static uint8_t *s_rx_buf = NULL;

static esp_err_t spi_write_bytes(uint8_t reg, const uint8_t *data, size_t len) {
    if (!spi_dev || !s_tx_buf) return ESP_ERR_INVALID_STATE;
    if (len + 1 > 512) return ESP_ERR_INVALID_SIZE;

    s_tx_buf[0] = reg & 0x7F; // MSB 0 = write
    if (data && len > 0) {
        memcpy(&s_tx_buf[1], data, len);
    }

    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = s_tx_buf,
        .rx_buffer = NULL,
    };
    return spi_device_polling_transmit(spi_dev, &t);
}

static esp_err_t spi_read_bytes(uint8_t reg, uint8_t *data, size_t len) {
    if (!spi_dev || !s_tx_buf || !s_rx_buf) return ESP_ERR_INVALID_STATE;
    if (len + 1 > 512) return ESP_ERR_INVALID_SIZE;

    memset(s_tx_buf, 0, len + 1);
    s_tx_buf[0] = reg | 0x80; // MSB 1 = read

    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = s_tx_buf,
        .rx_buffer = s_rx_buf,
    };
    esp_err_t ret = spi_device_polling_transmit(spi_dev, &t);
    if (ret == ESP_OK && data) {
        // rx_buf[0] is during register address transmission
        // rx_buf[1..len] contains the valid returned register data
        memcpy(data, &s_rx_buf[1], len);
    }
    return ret;
}

static void write_reg(uint8_t reg, uint8_t val) {
    spi_write_bytes(reg, &val, 1);
    esp_rom_delay_us(100);
}

static uint8_t read_reg(uint8_t reg) {
    uint8_t val = 0;
    spi_read_bytes(reg, &val, 1);
    return val;
}

esp_err_t bmi270_init(void) {
    // 1. Allocate 32-bit DMA-capable buffers
    if (!s_tx_buf) s_tx_buf = heap_caps_malloc(512, MALLOC_CAP_DMA);
    if (!s_rx_buf) s_rx_buf = heap_caps_malloc(512, MALLOC_CAP_DMA);
    if (!s_tx_buf || !s_rx_buf) {
        ESP_LOGE(TAG, "Failed to allocate DMA SPI buffers");
        return ESP_ERR_NO_MEM;
    }

    // 2. Configure both CS pins as OUTPUT HIGH (Deselected)
    gpio_config_t cs_conf = {
        .pin_bit_mask = (1ULL << PMW3901_CS_PIN) | (1ULL << BMI270_CS_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cs_conf);
    gpio_set_level(PMW3901_CS_PIN, 1); // Pin 12 HIGH
    gpio_set_level(BMI270_CS_PIN, 1);  // Pin 46 HIGH
    esp_rom_delay_us(5000);

    // 3. Initialize SPI Bus (8 MHz, SPI_DMA_CH_AUTO)
    spi_bus_config_t buscfg = {
        .miso_io_num = IMU_SPI_MISO_PIN, // GPIO 43
        .mosi_io_num = IMU_SPI_MOSI_PIN, // GPIO 14
        .sclk_io_num = IMU_SPI_SCK_PIN,  // GPIO 44
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096 * 2,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(IMU_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8 * 1000 * 1000,  // 8 MHz (SPI_MASTER_FREQ_8M)
        .mode = 0,                           // SPI Mode 0
        .spics_io_num = BMI270_CS_PIN,       // GPIO 46 (PIN_CS)
        .queue_size = 10,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(IMU_SPI_HOST, &devcfg, &spi_dev));

    // 4. BMI270 SPI Mode Select Protocol:
    // Device powers up in I2C mode. A dummy read switches it into SPI mode.
    read_reg(BMI270_REG_CHIP_ID);
    esp_rom_delay_us(2000);

    // 6. Soft Reset
    write_reg(BMI270_REG_CMD, BMI270_CMD_SOFTRESET);
    vTaskDelay(pdMS_TO_TICKS(100)); // 100ms reset delay

    // Re-enable SPI mode after reset
    read_reg(BMI270_REG_CHIP_ID);
    esp_rom_delay_us(1000);

    // 7. Microcode Upload (Bosch BMI270 ASIP Config Initialization)
    write_reg(BMI270_REG_PWR_CONF, 0x00); // Disable advanced power save
    esp_rom_delay_us(500);
    write_reg(BMI270_REG_INIT_CTRL, 0x00); // Prepare for burst write

    // Transfer configuration blob in burst to INIT_DATA register
    spi_write_bytes(BMI270_REG_INIT_DATA, bmi270_config_file, sizeof(bmi270_config_file));
    vTaskDelay(pdMS_TO_TICKS(10));

    // Complete microcode init
    write_reg(BMI270_REG_INIT_CTRL, 0x01);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Check internal status (bit 0 = init_ok)
    uint8_t status = read_reg(BMI270_REG_INTERNAL_STATUS);
    ESP_LOGI(TAG, "BMI270 Internal Status: 0x%02X (%s)", status, (status & 0x01) ? "INIT OK" : "INIT FAILED");

    // 8. Configure Accelerometer: +/- 8g, High Performance, OSR2, 1600Hz ODR
    write_reg(BMI270_REG_ACC_CONF, (1 << 7) | (1 << 4) | 0x0C);
    write_reg(BMI270_REG_ACC_RANGE, 0x02); // 8g (4096 LSB/g)

    // 9. Configure Gyroscope: +/- 2000 dps, High Performance, OSR4, 3200Hz ODR
    write_reg(BMI270_REG_GYR_CONF, (1 << 7) | (1 << 6) | (0 << 4) | 0x0D);
    write_reg(BMI270_REG_GYR_RANGE, 0x00); // 2000 dps (16.4 LSB/dps)

    // 10. Enable Power
    write_reg(BMI270_REG_PWR_CONF, 0x02); // Normal mode
    write_reg(BMI270_REG_PWR_CTRL, 0x0E); // Enable Accel + Gyro + Temp
    esp_rom_delay_us(5000);

    ESP_LOGI(TAG, "BMI270 Fully Operational (8g / 2000dps)");
    return ESP_OK;
}

esp_err_t bmi270_read(imu_data_t *data) {
    if (!data) return ESP_ERR_INVALID_ARG;

    uint8_t raw[12];
    esp_err_t ret = spi_read_bytes(BMI270_REG_DATA, raw, 12);
    if (ret != ESP_OK) return ret;

    // Convert raw 16-bit little-endian integers
    int16_t ax_raw = (int16_t)(raw[0] | (raw[1] << 8));
    int16_t ay_raw = (int16_t)(raw[2] | (raw[3] << 8));
    int16_t az_raw = (int16_t)(raw[4] | (raw[5] << 8));

    int16_t gx_raw = (int16_t)(raw[6] | (raw[7] << 8));
    int16_t gy_raw = (int16_t)(raw[8] | (raw[9] << 8));
    int16_t gz_raw = (int16_t)(raw[10] | (raw[11] << 8));

    // Scale factors: +/- 8g -> 4096 LSB/g; +/- 2000 dps -> 16.4 LSB/dps
    data->ax_g = (float)ax_raw / 4096.0f;
    data->ay_g = (float)ay_raw / 4096.0f;
    data->az_g = (float)az_raw / 4096.0f;

    data->gx_dps = (float)gx_raw / 16.4f;
    data->gy_dps = (float)gy_raw / 16.4f;
    data->gz_dps = (float)gz_raw / 16.4f;

    const float DEG_TO_RAD = (float)M_PI / 180.0f;
    data->gx_rads = data->gx_dps * DEG_TO_RAD;
    data->gy_rads = data->gy_dps * DEG_TO_RAD;
    data->gz_rads = data->gz_dps * DEG_TO_RAD;

    return ESP_OK;
}
