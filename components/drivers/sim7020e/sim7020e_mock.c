// Mock SIM7020E: returns the cell last set by the test/simulator.
#include "sim7020e.h"

#include <string.h>

static CellFix_t s_cell;

void sim7020e_mock_set_cell(const CellFix_t *cell) { s_cell = *cell; }

static esp_err_t mock_init(void) {
    memset(&s_cell, 0, sizeof(s_cell));
    s_cell.mcc = 425; // default: Israel
    s_cell.mnc = 1;
    s_cell.valid = true;
    return ESP_OK;
}

static esp_err_t mock_scan(CellFix_t *out) {
    *out = s_cell;
    return ESP_OK;
}

static esp_err_t mock_deinit(void) { return ESP_OK; }

static const Sim7020eDriver_t s_mock = {
    .init = mock_init,
    .scan = mock_scan,
    .deinit = mock_deinit,
};

const Sim7020eDriver_t *sim7020e_get_mock_driver(void) { return &s_mock; }
