// Real BMI270 backend over SPI @ 8MHz. ESP32 build only.
#include "bmi270.h"

#ifdef ESP_PLATFORM

#include <string.h>
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BMI270_HOST       SPI2_HOST
#define BMI270_CS_PIN     5
#define BMI270_SCLK_PIN   18
#define BMI270_MOSI_PIN   23
#define BMI270_MISO_PIN   19
#define BMI270_CLOCK_HZ   (8 * 1000 * 1000)

#define REG_CHIP_ID       0x00
#define REG_ACC_X_LSB     0x0C
#define REG_GYR_X_LSB     0x12
#define REG_PWR_CTRL      0x7D
#define REG_CMD           0x7E

// Full-scale ranges (defaults): +/-16g accel, +/-2000 dps gyro.
#define ACC_SCALE  ((16.0f / 32768.0f) * 9.81f)
#define GYR_SCALE  (2000.0f / 32768.0f)

static const char *TAG = "bmi270";
static spi_device_handle_t s_dev;

static esp_err_t spi_read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    // BMI270 read: MSB set, one dummy byte after the address.
    uint8_t tx[2 + 12] = { (uint8_t)(reg | 0x80), 0x00 };
    uint8_t rx[2 + 12] = {0};
    spi_transaction_t t = {
        .length = (2 + len) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_polling_transmit(s_dev, &t);
    if (err != ESP_OK) return err;
    memcpy(buf, &rx[2], len); // skip addr echo + dummy
    return ESP_OK;
}

static esp_err_t spi_write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    return spi_device_polling_transmit(s_dev, &t);
}

static esp_err_t bmi_init(void) {
    spi_bus_config_t bus = {
        .mosi_io_num = BMI270_MOSI_PIN,
        .miso_io_num = BMI270_MISO_PIN,
        .sclk_io_num = BMI270_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    esp_err_t err = spi_bus_initialize(BMI270_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    spi_device_interface_config_t dev = {
        .clock_speed_hz = BMI270_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = BMI270_CS_PIN,
        .queue_size = 1,
    };
    err = spi_bus_add_device(BMI270_HOST, &dev, &s_dev);
    if (err != ESP_OK) return err;

    // Dummy read toggles BMI270 into SPI mode after power-on.
    uint8_t id;
    spi_read_regs(REG_CHIP_ID, &id, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
    // Enable accel + gyro.
    spi_write_reg(REG_PWR_CTRL, 0x0E);
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGI(TAG, "BMI270 init complete");
    return ESP_OK;
}

static esp_err_t bmi_who_am_i(uint8_t *chip_id_out) {
    return spi_read_regs(REG_CHIP_ID, chip_id_out, 1);
}

static int16_t le16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static esp_err_t bmi_read(ImuData_t *out) {
    uint8_t acc[6], gyr[6];
    esp_err_t err = spi_read_regs(REG_ACC_X_LSB, acc, 6);
    if (err != ESP_OK) return err;
    err = spi_read_regs(REG_GYR_X_LSB, gyr, 6);
    if (err != ESP_OK) return err;

    out->accel_x_mss = le16(&acc[0]) * ACC_SCALE;
    out->accel_y_mss = le16(&acc[2]) * ACC_SCALE;
    out->gyro_z_dps  = le16(&gyr[4]) * GYR_SCALE;

    // Stationary heuristic: near-1g total accel and tiny yaw rate.
    float az = le16(&acc[4]) * ACC_SCALE;
    float mag = out->accel_x_mss * out->accel_x_mss +
                out->accel_y_mss * out->accel_y_mss + az * az;
    out->is_stationary = (mag > 90.0f && mag < 106.0f) &&
                         (out->gyro_z_dps > -1.0f && out->gyro_z_dps < 1.0f);
    return ESP_OK;
}

static esp_err_t bmi_deinit(void) {
    return spi_bus_remove_device(s_dev);
}

static const Bmi270Driver_t s_spi = {
    .init = bmi_init,
    .who_am_i = bmi_who_am_i,
    .read = bmi_read,
    .deinit = bmi_deinit,
};

const Bmi270Driver_t *bmi270_get_spi_driver(void) { return &s_spi; }

#endif // ESP_PLATFORM
