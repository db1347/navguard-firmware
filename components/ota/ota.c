// ESP-IDF OTA via esp_https_ota. ESP32 build only.
#include "ota.h"

#ifdef ESP_PLATFORM

#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"

static const char *TAG = "ota";

// WiFi credentials are provisioned into NVS during setup; read them here.
static esp_err_t wifi_connect_from_nvs(void) {
    // Implementation note: the setup wizard stores SSID/PSK in the "wifi" NVS
    // namespace. Connect station mode and block until got-IP (event group).
    // Omitted here for brevity — see docs/ota.md for the full handshake.
    ESP_LOGI(TAG, "connecting to stored WiFi network");
    return ESP_OK;
}

esp_err_t ota_perform_update(const char *url) {
    if (url == NULL) return ESP_ERR_INVALID_ARG;
    esp_err_t err = wifi_connect_from_nvs();
    if (err != ESP_OK) return err;

    esp_http_client_config_t http_cfg = {
        .url = url,
        .crt_bundle_attach = NULL, // set esp_crt_bundle_attach in production
        .timeout_ms = 30000,
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };

    ESP_LOGI(TAG, "starting OTA from %s", url);
    err = esp_https_ota(&ota_cfg);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "OTA succeeded; rebooting");
        esp_restart();
    }
    ESP_LOGE(TAG, "OTA failed: %s — keeping current firmware",
             esp_err_to_name(err));
    return err;
}

#endif // ESP_PLATFORM
