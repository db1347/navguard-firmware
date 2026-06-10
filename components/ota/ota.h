// WiFi OTA firmware update. Triggered over BLE by the Android app; downloads a
// new binary over HTTPS (Firebase Hosting) and flashes the inactive partition.
#ifndef NAVGUARD_OTA_H
#define NAVGUARD_OTA_H

#include "navguard_compat.h"

#ifdef __cplusplus
extern "C" {
#endif

// Connect to the stored WiFi network, download `url`, flash, and reboot on
// success. Blocks; run from a dedicated task. Returns an error without
// rebooting if the download or validation fails (current firmware kept).
esp_err_t ota_perform_update(const char *url);

#ifdef __cplusplus
}
#endif

#endif // NAVGUARD_OTA_H
