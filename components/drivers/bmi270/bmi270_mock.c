// Mock BMI270: serves the last sample pushed by the test/simulator.
#include "bmi270.h"

#include <string.h>

static ImuData_t s_sample;

void bmi270_mock_push(const ImuData_t *sample) {
    s_sample = *sample;
}

static esp_err_t mock_init(void) {
    memset(&s_sample, 0, sizeof(s_sample));
    s_sample.is_stationary = true;
    return ESP_OK;
}

static esp_err_t mock_who_am_i(uint8_t *chip_id_out) {
    *chip_id_out = BMI270_CHIP_ID;
    return ESP_OK;
}

static esp_err_t mock_read(ImuData_t *out) {
    *out = s_sample;
    return ESP_OK;
}

static esp_err_t mock_deinit(void) { return ESP_OK; }

static const Bmi270Driver_t s_mock = {
    .init = mock_init,
    .who_am_i = mock_who_am_i,
    .read = mock_read,
    .deinit = mock_deinit,
};

const Bmi270Driver_t *bmi270_get_mock_driver(void) { return &s_mock; }
