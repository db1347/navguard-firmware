// BLE GATT server exposing the Nordic UART Service (NUS). The ESP32-S3 is the
// peripheral; the Android phone is the central. NMEA sentences are streamed on
// the TX characteristic via notifications; OTA triggers arrive on RX.
#ifndef NAVGUARD_BLE_SERVER_H
#define NAVGUARD_BLE_SERVER_H

#include <stddef.h>
#include "navguard_compat.h"

#ifdef __cplusplus
extern "C" {
#endif

// Nordic UART Service UUIDs (dev plan Section 4.5).
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // notify
#define NUS_RX_CHAR_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // write

// Invoked when the central writes to the RX characteristic (e.g. OTA trigger).
typedef void (*ble_rx_cb_t)(const uint8_t *data, size_t len);

// Start advertising as "NavGuard-XXXX". rx_cb may be NULL.
esp_err_t ble_server_start(ble_rx_cb_t rx_cb);

// Push one NMEA sentence to the subscribed central. Safe to call at 10Hz.
esp_err_t ble_server_notify_nmea(const char *sentence, size_t len);

// True once a central has subscribed to TX notifications.
bool ble_server_is_subscribed(void);

#ifdef __cplusplus
}
#endif

#endif // NAVGUARD_BLE_SERVER_H
