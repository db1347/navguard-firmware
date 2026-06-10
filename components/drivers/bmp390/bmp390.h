// BMP390 barometer driver (I2C @ 400kHz, addr 0x77). Outputs altitude (m).
#ifndef NAVGUARD_BMP390_H
#define NAVGUARD_BMP390_H

#include <stdint.h>
#include "navguard_compat.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMP390_I2C_ADDR 0x77
#define BMP390_CHIP_ID  0x60

typedef struct {
    esp_err_t (*init)(void);
    esp_err_t (*read_altitude_m)(float *alt_m);
    esp_err_t (*deinit)(void);
} Bmp390Driver_t;

const Bmp390Driver_t *bmp390_get_i2c_driver(void);

const Bmp390Driver_t *bmp390_get_mock_driver(void);
void bmp390_mock_set_altitude(float alt_m);

#ifdef __cplusplus
}
#endif

#endif // NAVGUARD_BMP390_H
