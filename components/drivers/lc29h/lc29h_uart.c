// Real LC29H-EA backend over UART1 @ 115200. Compiled on the ESP32 build only.
// On desktop this file is excluded (guarded by ESP_PLATFORM) so the mock backend
// provides the driver.
#include "lc29h.h"

#ifdef ESP_PLATFORM

#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"

#define LC29H_UART        UART_NUM_1
#define LC29H_TX_PIN      17
#define LC29H_RX_PIN      18
#define LC29H_BAUD        115200
#define LC29H_BUF_SIZE    (1024 * 4)
#define LC29H_LINE_MAX    128

static const char *TAG = "lc29h";
static NmeaFix_t s_accum;
static char s_line[LC29H_LINE_MAX];
static size_t s_line_len;

static esp_err_t uart_init(void) {
    uart_config_t cfg = {
        .baud_rate = LC29H_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(LC29H_UART, LC29H_BUF_SIZE, 0, 0, NULL, 0);
    if (err != ESP_OK) return err;
    uart_param_config(LC29H_UART, &cfg);
    uart_set_pin(LC29H_UART, LC29H_TX_PIN, LC29H_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    memset(&s_accum, 0, sizeof(s_accum));
    s_line_len = 0;
    nmea_parser_reset();
    ESP_LOGI(TAG, "LC29H UART ready on UART%d", LC29H_UART);
    return ESP_OK;
}

static esp_err_t uart_read_fix(NmeaFix_t *out) {
    uint8_t byte;
    // Drain available bytes, assembling lines; return on a completed GGA.
    for (;;) {
        int n = uart_read_bytes(LC29H_UART, &byte, 1, pdMS_TO_TICKS(50));
        if (n <= 0) return ESP_ERR_TIMEOUT;
        if (byte == '\n' || byte == '\r') {
            if (s_line_len == 0) continue;
            s_line[s_line_len] = '\0';
            s_line_len = 0;
            if (s_line[0] != '$') continue;
            if (nmea_parse_sentence(s_line, &s_accum) != NMEA_PARSE_OK) continue;
            if (strstr(s_line, "GGA") != NULL && s_accum.valid) {
                *out = s_accum;
                return ESP_OK;
            }
        } else if (s_line_len < LC29H_LINE_MAX - 1) {
            s_line[s_line_len++] = (char)byte;
        } else {
            s_line_len = 0; // overflow: drop the line
        }
    }
}

static esp_err_t uart_set_rate(uint8_t hz) {
    // PQTMCFGNMEADP configures output rate and persists with the 'W' flag.
    // 1 => 1Hz, 10 => 10Hz divider. Command shape is module-specific.
    char cmd[64];
    int divider = (hz >= 10) ? 1 : 1;
    snprintf(cmd, sizeof(cmd), "$PQTMCFGNMEADP,W,%d,1,1,1,1,1,1", divider);
    // append checksum
    uint8_t cs = 0;
    for (const char *p = cmd + 1; *p; p++) cs ^= (uint8_t)*p;
    char full[80];
    snprintf(full, sizeof(full), "%s*%02X\r\n", cmd, cs);
    uart_write_bytes(LC29H_UART, full, strlen(full));
    return ESP_OK;
}

static esp_err_t uart_deinit(void) {
    return uart_driver_delete(LC29H_UART);
}

static const Lc29hDriver_t s_uart_driver = {
    .init = uart_init,
    .read = uart_read_fix,
    .set_rate_hz = uart_set_rate,
    .deinit = uart_deinit,
};

const Lc29hDriver_t *lc29h_get_uart_driver(void) {
    return &s_uart_driver;
}

#endif // ESP_PLATFORM
