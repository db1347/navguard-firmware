// Real SIM7020E backend over UART2 @ 9600. ESP32 build only.
#include "sim7020e.h"

#ifdef ESP_PLATFORM

#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SIM_UART       UART_NUM_2
#define SIM_TX_PIN     16
#define SIM_RX_PIN     4
#define SIM_PWRKEY_PIN 15
#define SIM_BAUD       9600
#define SIM_BUF_SIZE   2048

static const char *TAG = "sim7020e";

static int send_at(const char *cmd, char *resp, size_t resp_len, int timeout_ms) {
    uart_flush_input(SIM_UART);
    uart_write_bytes(SIM_UART, cmd, strlen(cmd));
    uart_write_bytes(SIM_UART, "\r\n", 2);
    int n = uart_read_bytes(SIM_UART, (uint8_t *)resp, resp_len - 1,
                            pdMS_TO_TICKS(timeout_ms));
    if (n < 0) n = 0;
    resp[n] = '\0';
    return n;
}

static esp_err_t sim_init(void) {
    uart_config_t cfg = {
        .baud_rate = SIM_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(SIM_UART, SIM_BUF_SIZE, 0, 0, NULL, 0);
    if (err != ESP_OK) return err;
    uart_param_config(SIM_UART, &cfg);
    uart_set_pin(SIM_UART, SIM_TX_PIN, SIM_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Pulse PWRKEY to power the modem on.
    gpio_set_direction(SIM_PWRKEY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SIM_PWRKEY_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(SIM_PWRKEY_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    char resp[128];
    send_at("AT", resp, sizeof(resp), 1000);
    if (strstr(resp, "OK") == NULL) {
        ESP_LOGW(TAG, "modem not responding to AT");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "SIM7020E ready");
    return ESP_OK;
}

static esp_err_t sim_scan(CellFix_t *out) {
    char resp[512];
    int n = send_at("AT+QENG=\"servingcell\"", resp, sizeof(resp), 2000);
    if (n <= 0) return ESP_ERR_TIMEOUT;
    if (!sim7020e_parse_servingcell(resp, out)) return ESP_ERR_NOT_FOUND;
    return ESP_OK;
}

static esp_err_t sim_deinit(void) {
    return uart_driver_delete(SIM_UART);
}

static const Sim7020eDriver_t s_uart = {
    .init = sim_init,
    .scan = sim_scan,
    .deinit = sim_deinit,
};

const Sim7020eDriver_t *sim7020e_get_uart_driver(void) { return &s_uart; }

#endif // ESP_PLATFORM
