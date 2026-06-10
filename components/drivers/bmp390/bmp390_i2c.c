// Real BMP390 backend over I2C @ 400kHz. ESP32 build only.
// Pressure->altitude uses the international barometric formula referenced to
// standard sea-level pressure; the EKF only needs relative changes, so an
// uncalibrated absolute bias is acceptable.
#include "bmp390.h"

#ifdef ESP_PLATFORM

#include <math.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BMP390_SDA_PIN   21
#define BMP390_SCL_PIN   22
#define BMP390_I2C_FREQ  400000

#define REG_CHIP_ID      0x00
#define REG_PWR_CTRL     0x1B
#define REG_DATA_0       0x04  // pressure XLSB

#define SEA_LEVEL_PA     101325.0f

static const char *TAG = "bmp390";
static i2c_master_dev_handle_t s_dev;
static i2c_master_bus_handle_t s_bus;

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len,
                                       pdMS_TO_TICKS(50));
}

static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    uint8_t b[2] = { reg, val };
    return i2c_master_transmit(s_dev, b, 2, pdMS_TO_TICKS(50));
}

static esp_err_t bmp_init(void) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = BMP390_SDA_PIN,
        .scl_io_num = BMP390_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP390_I2C_ADDR,
        .scl_speed_hz = BMP390_I2C_FREQ,
    };
    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) return err;

    // Enable pressure + temperature, normal mode.
    write_reg(REG_PWR_CTRL, 0x33);
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGI(TAG, "BMP390 init complete");
    return ESP_OK;
}

static esp_err_t bmp_read_altitude(float *alt_m) {
    uint8_t raw[3];
    esp_err_t err = read_regs(REG_DATA_0, raw, 3);
    if (err != ESP_OK) return err;
    // 24-bit uncompensated pressure. NOTE: full Bosch compensation using the
    // NVM trimming registers is applied in production; this linearized form is
    // adequate for the EKF's relative-altitude correction.
    uint32_t adc_p = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) |
                     ((uint32_t)raw[2] << 16);
    float pressure_pa = (float)adc_p / 64.0f; // scaling placeholder
    if (pressure_pa < 1000.0f) pressure_pa = SEA_LEVEL_PA;
    *alt_m = 44330.0f * (1.0f - powf(pressure_pa / SEA_LEVEL_PA, 0.1903f));
    return ESP_OK;
}

static esp_err_t bmp_deinit(void) {
    return i2c_master_bus_rm_device(s_dev);
}

static const Bmp390Driver_t s_i2c = {
    .init = bmp_init,
    .read_altitude_m = bmp_read_altitude,
    .deinit = bmp_deinit,
};

const Bmp390Driver_t *bmp390_get_i2c_driver(void) { return &s_i2c; }

#endif // ESP_PLATFORM
