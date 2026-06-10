// BMI270 IMU driver (SPI). Outputs scaled gyro (deg/s) + accel (m/s^2).
#ifndef NAVGUARD_BMI270_H
#define NAVGUARD_BMI270_H

#include <stdint.h>
#include "navguard_compat.h"
#include "ekf.h" // ImuData_t

#ifdef __cplusplus
extern "C" {
#endif

#define BMI270_CHIP_ID 0x24

typedef struct {
    esp_err_t (*init)(void);
    esp_err_t (*who_am_i)(uint8_t *chip_id_out); // expect BMI270_CHIP_ID
    esp_err_t (*read)(ImuData_t *out);
    esp_err_t (*deinit)(void);
} Bmi270Driver_t;

const Bmi270Driver_t *bmi270_get_spi_driver(void);

// Mock IMU: returns the values most recently pushed via bmi270_mock_push().
const Bmi270Driver_t *bmi270_get_mock_driver(void);
void bmi270_mock_push(const ImuData_t *sample);

#ifdef __cplusplus
}
#endif

#endif // NAVGUARD_BMI270_H
