// SIM7020E NB-IoT/Cat-M1 cell modem driver (UART2 @ 9600, AT commands).
// Used only to read the serving cell for the spoofing detector's anchor —
// no data plan required.
#ifndef NAVGUARD_SIM7020E_H
#define NAVGUARD_SIM7020E_H

#include "navguard_compat.h"
#include "ekf.h" // CellFix_t

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    esp_err_t (*init)(void);
    esp_err_t (*scan)(CellFix_t *out);  // AT+QENG="servingcell"
    esp_err_t (*deinit)(void);
} Sim7020eDriver_t;

const Sim7020eDriver_t *sim7020e_get_uart_driver(void);

const Sim7020eDriver_t *sim7020e_get_mock_driver(void);
void sim7020e_mock_set_cell(const CellFix_t *cell);

// Pure parser for AT+QENG="servingcell" responses — desktop-testable.
// Returns true and fills *out on success.
bool sim7020e_parse_servingcell(const char *resp, CellFix_t *out);

#ifdef __cplusplus
}
#endif

#endif // NAVGUARD_SIM7020E_H
