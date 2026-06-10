// Mock LC29H backend: replays a recorded NMEA file. One read() returns one
// accumulated fix (sentences are merged until a GGA completes the pair).
#include "lc29h.h"

#include <stdio.h>
#include <string.h>

static const char *s_nmea_path = NULL;
static FILE *s_file = NULL;
static NmeaFix_t s_accum;

static esp_err_t mock_init(void) {
    if (s_nmea_path == NULL) return ESP_ERR_INVALID_ARG;
    s_file = fopen(s_nmea_path, "r");
    if (s_file == NULL) return ESP_ERR_NOT_FOUND;
    memset(&s_accum, 0, sizeof(s_accum));
    nmea_parser_reset();
    return ESP_OK;
}

static esp_err_t mock_read(NmeaFix_t *out) {
    if (s_file == NULL) return ESP_FAIL;
    char line[128];
    while (fgets(line, sizeof(line), s_file) != NULL) {
        // strip CR/LF
        line[strcspn(line, "\r\n")] = '\0';
        if (line[0] != '$') continue;
        NmeaParseResult_t r = nmea_parse_sentence(line, &s_accum);
        if (r != NMEA_PARSE_OK) continue;
        // A GGA sentence completes a fix epoch (RMC then GGA per cycle).
        if (strstr(line, "GGA") != NULL && s_accum.valid) {
            *out = s_accum;
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND; // EOF
}

static esp_err_t mock_set_rate_hz(uint8_t hz) {
    (void)hz; // replay rate is controlled by the caller's loop
    return ESP_OK;
}

static esp_err_t mock_deinit(void) {
    if (s_file != NULL) {
        fclose(s_file);
        s_file = NULL;
    }
    return ESP_OK;
}

static const Lc29hDriver_t s_mock_driver = {
    .init = mock_init,
    .read = mock_read,
    .set_rate_hz = mock_set_rate_hz,
    .deinit = mock_deinit,
};

const Lc29hDriver_t *lc29h_get_mock_driver(const char *nmea_file) {
    s_nmea_path = nmea_file;
    return &s_mock_driver;
}
