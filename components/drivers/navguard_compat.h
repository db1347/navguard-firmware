// Minimal ESP-IDF type shim so driver interfaces compile in the desktop
// test build. On the ESP32 build the real esp_err.h is used instead.
#ifndef NAVGUARD_COMPAT_H
#define NAVGUARD_COMPAT_H

#ifdef ESP_PLATFORM
#include "esp_err.h"
#else
typedef int esp_err_t;
#define ESP_OK              0
#define ESP_FAIL            (-1)
#define ESP_ERR_TIMEOUT     0x107
#define ESP_ERR_NOT_FOUND   0x105
#define ESP_ERR_INVALID_ARG 0x102
#endif

#endif // NAVGUARD_COMPAT_H
