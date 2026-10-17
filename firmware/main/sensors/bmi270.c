#include "bmi270.h"
#include "bmi270_config.h"
#include "../config.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <math.h>

static const char *TAG = "BMI270";
static spi_device_handle_t spi_dev = NULL;
static SemaphoreHandle_t s_spi_mutex = NULL;

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

// Calibration offsets
static float s_acc_offset[3] = { 0.0f, 0.0f, 0.0f };
static float s_gyro_offset[3] = { 0.0f, 0.0f, 0.0f };
static bool s_calibrated = false;

// DMA-safe memory buffers
static uint8_t *s_tx_buf = NULL;
static uint8_t *s_rx_buf = NULL;

static esp_err_t spi_write_bytes(uint8_t reg, const uint8_t *data, size_t len) {
    if (!spi_dev || !s_tx_buf) return ESP_ERR_INVALID_STATE;
    if (len + 1 > 512) return ESP_ERR_INVALID_SIZE;

    if (s_spi_mutex) xSemaphoreTake(s_spi_mutex, portMAX_DELAY);

    s_tx_buf[0] = reg & 0x7F; // MSB 0 = write
    if (data && len > 0) {
        memcpy(&s_tx_buf[1], data, len);
    }

    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = s_tx_buf,
        .rx_buffer = NULL,
    };
    esp_err_t ret = spi_device_polling_transmit(spi_dev, &t);

    if (s_spi_mutex) xSemaphoreGive(s_spi_mutex);
    return ret;
}

static esp_err_t spi_read_bytes(uint8_t reg, uint8_t *data, size_t len) {
    if (!spi_dev || !s_tx_buf || !s_rx_buf) return ESP_ERR_INVALID_STATE;
    if (len + 2 > 512) return ESP_ERR_INVALID_SIZE;

    if (s_spi_mutex) xSemaphoreTake(s_spi_mutex, portMAX_DELAY);

    memset(s_tx_buf, 0, len + 2);
    s_tx_buf[0] = reg | 0x80; // MSB 1 = read

    spi_transaction_t t = {
        .length = (len + 2) * 8, // 1 address byte + 1 dummy turnaround byte + len data bytes
        .tx_buffer = s_tx_buf,
        .rx_buffer = s_rx_buf,
    };
    esp_err_t ret = spi_device_polling_transmit(spi_dev, &t);
    if (ret == ESP_OK && data) {
        // rx_buf[0] = during address transmission (discard)
        // rx_buf[1] = BMI270 SPI turnaround dummy byte (discard)
        // rx_buf[2..len+1] = valid register data!
        memcpy(data, &s_rx_buf[2], len);
    }

    if (s_spi_mutex) xSemaphoreGive(s_spi_mutex);
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

void bmi270_calibrate(void) {
    float ax_sum = 0.0f, ay_sum = 0.0f, az_sum = 0.0f;
    float gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;
    const int SAMPLES = 50;
    int valid = 0;

    for (int i = 0; i < SAMPLES; i++) {
        uint8_t raw[12];
        if (spi_read_bytes(BMI270_REG_DATA, raw, 12) == ESP_OK) {
            int16_t ax = (int16_t)(raw[0] | (raw[1] << 8));
            int16_t ay = (int16_t)(raw[2] | (raw[3] << 8));
            int16_t az = (int16_t)(raw[4] | (raw[5] << 8));
            int16_t gx = (int16_t)(raw[6] | (raw[7] << 8));
            int16_t gy = (int16_t)(raw[8] | (raw[9] << 8));
            int16_t gz = (int16_t)(raw[10] | (raw[11] << 8));

            ax_sum += (float)ax / 4096.0f;
            ay_sum += (float)ay / 4096.0f;
            az_sum += -((float)az / 4096.0f); // Invert Z to map to body frame (+1g upward)
            gx_sum += (float)gx / 16.4f;
            gy_sum += (float)gy / 16.4f;
            gz_sum += -((float)gz / 16.4f);
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (valid > 0) {
        s_gyro_offset[0] = gx_sum / valid;
        s_gyro_offset[1] = gy_sum / valid;
        s_gyro_offset[2] = gz_sum / valid;

        // Gravity target at rest: Z = +1.0g, X = 0.0g, Y = 0.0g
        s_acc_offset[0] = ax_sum / valid;
        s_acc_offset[1] = ay_sum / valid;
        s_acc_offset[2] = (az_sum / valid) - 1.0f;
        s_calibrated = true;
    }
}

esp_err_t bmi270_init(void) {
    if (!s_spi_mutex) s_spi_mutex = xSemaphoreCreateMutex();

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
        .clock_speed_hz = 8 * 1000 * 1000,  // 8 MHz
        .mode = 0,                           // SPI Mode 0
        .spics_io_num = BMI270_CS_PIN,       // GPIO 46 (PIN_CS)
        .queue_size = 10,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(IMU_SPI_HOST, &devcfg, &spi_dev));

    // 4. BMI270 SPI Mode Select Protocol:
    // Device powers up in I2C mode. A dummy read switches it into SPI mode.
    read_reg(BMI270_REG_CHIP_ID);
    esp_rom_delay_us(2000);

    // 5. Read Chip ID
    uint8_t chip_id = read_reg(BMI270_REG_CHIP_ID);
    ESP_LOGI(TAG, "BMI270 Read Chip ID: 0x%02X (Expected: 0x24)", chip_id);

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

    // 11. Initial resting calibration
    bmi270_calibrate();

    ESP_LOGI(TAG, "BMI270 Fully Operational (8g / 2000dps, Zeroed)");
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
    float ax = (float)ax_raw / 4096.0f;
    float ay = (float)ay_raw / 4096.0f;
    float az = -((float)az_raw / 4096.0f); // Invert Z to map to body frame (+1g upward)

    float gx = (float)gx_raw / 16.4f;
    float gy = (float)gy_raw / 16.4f;
    float gz = -((float)gz_raw / 16.4f);

    // Apply resting calibration offsets
    data->ax_g = ax - s_acc_offset[0];
    data->ay_g = ay - s_acc_offset[1];
    data->az_g = az - s_acc_offset[2];

    data->gx_dps = gx - s_gyro_offset[0];
    data->gy_dps = gy - s_gyro_offset[1];
    data->gz_dps = gz - s_gyro_offset[2];

    const float DEG_TO_RAD = (float)M_PI / 180.0f;
    data->gx_rads = data->gx_dps * DEG_TO_RAD;
    data->gy_rads = data->gy_dps * DEG_TO_RAD;
    data->gz_rads = data->gz_dps * DEG_TO_RAD;

    return ESP_OK;
}
