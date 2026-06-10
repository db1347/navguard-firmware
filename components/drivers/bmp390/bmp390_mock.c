// Mock BMP390: returns the altitude last set by the test/simulator.
#include "bmp390.h"

static float s_alt_m = 30.0f;

void bmp390_mock_set_altitude(float alt_m) { s_alt_m = alt_m; }

static esp_err_t mock_init(void) { return ESP_OK; }

static esp_err_t mock_read(float *alt_m) {
    *alt_m = s_alt_m;
    return ESP_OK;
}

static esp_err_t mock_deinit(void) { return ESP_OK; }

static const Bmp390Driver_t s_mock = {
    .init = mock_init,
    .read_altitude_m = mock_read,
    .deinit = mock_deinit,
};

const Bmp390Driver_t *bmp390_get_mock_driver(void) { return &s_mock; }
