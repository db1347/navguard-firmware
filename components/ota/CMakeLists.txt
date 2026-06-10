idf_component_register(
    SRCS "ota.c"
    INCLUDE_DIRS "."
    REQUIRES app_update esp_https_ota esp_http_client esp_wifi nvs_flash)
