#include "bmi270.h"
#include "../config.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include <string.h>
#include <math.h>

static const char *TAG = "BMI270";
static spi_device_handle_t spi_dev;

#define BMI270_CHIP_ID_REG   0x00
#define BMI270_DATA_REG      0x0C
#define BMI270_PWR_CONF_REG  0x7C
#define BMI270_PWR_CTRL_REG  0x7D
#define BMI270_ACC_CONF_REG  0x40
#define BMI270_GYR_CONF_REG  0x42

static esp_err_t bmi270_write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx_data[2] = { reg & 0x7F, val }; // MSB 0 = write
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
    };
    return spi_device_transmit(spi_dev, &t);
}

static esp_err_t bmi270_read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    uint8_t tx[len + 2];
    uint8_t rx[len + 2];
    memset(tx, 0, sizeof(tx));
    tx[0] = reg | 0x80; // MSB 1 = read

    spi_transaction_t t = {
        .length = (len + 2) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t ret = spi_device_transmit(spi_dev, &t);
    if (ret == ESP_OK) {
        // First byte after dummy byte is valid data
        memcpy(buf, &rx[2], len);
    }
    return ret;
}

esp_err_t bmi270_init(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num = IMU_SPI_MISO_PIN,
        .mosi_io_num = IMU_SPI_MOSI_PIN,
        .sclk_io_num = IMU_SPI_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(IMU_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // 10 MHz
        .mode = 0,                          // SPI Mode 0
        .spics_io_num = BMI270_CS_PIN,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(IMU_SPI_HOST, &devcfg, &spi_dev));

    // Disable power-save mode
    bmi270_write_reg(BMI270_PWR_CONF_REG, 0x00);
    esp_rom_delay_us(500);

    // Read chip ID
    uint8_t chip_id = 0;
    bmi270_read_regs(BMI270_CHIP_ID_REG, &chip_id, 1);
    ESP_LOGI(TAG, "BMI270 detected, Chip ID: 0x%02X", chip_id);

    // Enable Accelerometer & Gyroscope
    bmi270_write_reg(BMI270_PWR_CTRL_REG, 0x0E); // Accel + Gyro + Temp
    esp_rom_delay_us(5000);

    // Accel Config: 1.6kHz ODR, normal bandwidth, +/- 8g
    bmi270_write_reg(BMI270_ACC_CONF_REG, 0xA8);

    // Gyro Config: 3.2kHz ODR, normal bandwidth, +/- 2000 dps
    bmi270_write_reg(BMI270_GYR_CONF_REG, 0xA9);

    ESP_LOGI(TAG, "BMI270 configured: Accel +/-8g, Gyro +/-2000dps");
    return ESP_OK;
}

esp_err_t bmi270_read(imu_data_t *data) {
    uint8_t raw[12];
    esp_err_t ret = bmi270_read_regs(BMI270_DATA_REG, raw, 12);
    if (ret != ESP_OK) return ret;

    // Convert raw 16-bit integers
    int16_t ax_raw = (int16_t)(raw[0] | (raw[1] << 8));
    int16_t ay_raw = (int16_t)(raw[2] | (raw[3] << 8));
    int16_t az_raw = (int16_t)(raw[4] | (raw[5] << 8));

    int16_t gx_raw = (int16_t)(raw[6] | (raw[7] << 8));
    int16_t gy_raw = (int16_t)(raw[8] | (raw[9] << 8));
    int16_t gz_raw = (int16_t)(raw[10] | (raw[11] << 8));

    // Scale factors: +/- 8g range -> 4096 LSB/g; +/- 2000 dps range -> 16.4 LSB/dps
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
