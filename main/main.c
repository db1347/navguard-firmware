// NavGuard IL firmware entry point.
//
// Five concurrent FreeRTOS tasks share one EkfContext_t guarded by a mutex:
//   - imu_task   400Hz  ekf_predict (dead reckoning)
//   - gps_task    10Hz  parse fix -> spoof_update -> ekf_update_gps
//   - cell_task    1Hz  serving-cell scan -> anchor for spoof detector
//   - baro_task    5Hz  ekf_update_baro
//   - output_task 10Hz  ekf_get_position -> NMEA build -> BLE notify
//
// Swap the mock drivers for the real backends (lc29h_get_uart_driver() etc.)
// once hardware arrives — the EKF and detector are unchanged.
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "ekf.h"
#include "spoof_detect.h"
#include "nmea_builder.h"
#include "ble_server.h"
#include "ota.h"

#include "lc29h.h"
#include "bmi270.h"
#include "bmp390.h"
#include "sim7020e.h"

static const char *TAG = "navguard";

static EkfContext_t s_ekf;
static SpoofDetector_t s_spoof;
static SemaphoreHandle_t s_ekf_mutex;

static const Lc29hDriver_t    *s_gps;
static const Bmi270Driver_t   *s_imu;
static const Bmp390Driver_t   *s_baro;
static const Sim7020eDriver_t *s_cell;

#define LOCK()   xSemaphoreTake(s_ekf_mutex, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(s_ekf_mutex)

static void imu_task(void *arg) {
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(3); // ~333-400Hz
    ImuData_t imu;
    TickType_t last = xTaskGetTickCount();
    while (1) {
        if (s_imu->read(&imu) == ESP_OK) {
            LOCK();
            ekf_predict(&s_ekf, &imu, 0.0025f);
            UNLOCK();
        }
        vTaskDelayUntil(&last, period);
    }
}

static void gps_task(void *arg) {
    (void)arg;
    NmeaFix_t fix;
    while (1) {
        esp_err_t err = s_gps->read(&fix);
        if (err == ESP_OK) {
            LOCK();
            spoof_update(&s_spoof, &s_ekf, &fix, NULL);
            ekf_update_gps(&s_ekf, &fix);
            UNLOCK();
        } else if (err == ESP_ERR_TIMEOUT) {
            LOCK();
            spoof_on_gps_lost(&s_spoof, &s_ekf); // tunnel / jam
            UNLOCK();
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
    }
}

static void cell_task(void *arg) {
    (void)arg;
    CellFix_t cell;
    while (1) {
        if (s_cell->scan(&cell) == ESP_OK) {
            LOCK();
            // Re-evaluate the most recent state with the cell anchor.
            NmeaFix_t pos;
            ekf_get_position(&s_ekf, &pos);
            spoof_update(&s_spoof, &s_ekf, &pos, &cell);
            UNLOCK();
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1Hz
    }
}

static void baro_task(void *arg) {
    (void)arg;
    float alt;
    while (1) {
        if (s_baro->read_altitude_m(&alt) == ESP_OK) {
            LOCK();
            ekf_update_baro(&s_ekf, alt);
            UNLOCK();
        }
        vTaskDelay(pdMS_TO_TICKS(200)); // 5Hz
    }
}

static void output_task(void *arg) {
    (void)arg;
    char gga[96], rmc[96];
    while (1) {
        NmeaFix_t pos;
        LOCK();
        ekf_get_position(&s_ekf, &pos);
        UNLOCK();
        if (pos.valid) {
            int n1 = nmea_build_gngga(&pos, gga, sizeof(gga));
            int n2 = nmea_build_gnrmc(&pos, rmc, sizeof(rmc));
            if (n1 > 0) ble_server_notify_nmea(gga, n1);
            if (n2 > 0) ble_server_notify_nmea(rmc, n2);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
    }
}

static void ble_rx_handler(const uint8_t *data, size_t len) {
    // The Android app writes an OTA trigger: "OTA <url>".
    if (len > 4 && strncmp((const char *)data, "OTA ", 4) == 0) {
        static char url[256];
        size_t n = len - 4 < sizeof(url) - 1 ? len - 4 : sizeof(url) - 1;
        memcpy(url, data + 4, n);
        url[n] = '\0';
        ESP_LOGI(TAG, "OTA trigger received: %s", url);
        // Run OTA off the BLE callback context.
        xTaskCreate((TaskFunction_t)ota_perform_update, "ota", 8192,
                    url, 4, NULL);
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "NavGuard IL firmware boot");

    s_ekf_mutex = xSemaphoreCreateMutex();
    ekf_init(&s_ekf);
    spoof_init(&s_spoof);

    // --- Driver selection ---------------------------------------------------
    // Mock backends boot without hardware (useful for BLE/Android bring-up).
    // Swap to the real backends below when components are connected.
    s_gps  = lc29h_get_mock_driver("/spiffs/tel_aviv_drive.nmea");
    s_imu  = bmi270_get_mock_driver();
    s_baro = bmp390_get_mock_driver();
    s_cell = sim7020e_get_mock_driver();
    //  s_gps  = lc29h_get_uart_driver();
    //  s_imu  = bmi270_get_spi_driver();
    //  s_baro = bmp390_get_i2c_driver();
    //  s_cell = sim7020e_get_uart_driver();

    s_gps->init();
    s_imu->init();
    s_baro->init();
    s_cell->init();

    uint8_t chip_id = 0;
    if (s_imu->who_am_i(&chip_id) == ESP_OK) {
        ESP_LOGI(TAG, "BMI270 chip id 0x%02X (expect 0x%02X)",
                 chip_id, BMI270_CHIP_ID);
    }

    ble_server_start(ble_rx_handler);

    xTaskCreate(imu_task,    "imu",    4096, NULL, 6, NULL);
    xTaskCreate(gps_task,    "gps",    4096, NULL, 5, NULL);
    xTaskCreate(cell_task,   "cell",   4096, NULL, 3, NULL);
    xTaskCreate(baro_task,   "baro",   4096, NULL, 3, NULL);
    xTaskCreate(output_task, "output", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "all tasks started");
}
