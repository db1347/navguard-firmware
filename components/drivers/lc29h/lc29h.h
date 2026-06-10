// LC29H-EA GPS module driver — same interface, two implementations:
// UART backend for hardware, mock backend reading a recorded NMEA file.
#ifndef NAVGUARD_LC29H_H
#define NAVGUARD_LC29H_H

#include <stdint.h>
#include "navguard_compat.h"
#include "nmea_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    esp_err_t (*init)(void);
    // Reads the next complete fix (a parsed GGA+RMC pair).
    // Returns ESP_OK, ESP_ERR_TIMEOUT (no new data) or ESP_ERR_NOT_FOUND (EOF on mock).
    esp_err_t (*read)(NmeaFix_t *out);
    esp_err_t (*set_rate_hz)(uint8_t hz);
    esp_err_t (*deinit)(void);
} Lc29hDriver_t;

// Real hardware driver (UART1 @ 115200). ESP32 build only.
const Lc29hDriver_t *lc29h_get_uart_driver(void);

// Mock driver: replays a recorded .nmea file at the configured rate.
// Desktop build (and on-target simulation) — call before init().
const Lc29hDriver_t *lc29h_get_mock_driver(const char *nmea_file);

#ifdef __cplusplus
}
#endif

#endif // NAVGUARD_LC29H_H
