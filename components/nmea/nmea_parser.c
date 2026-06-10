#include "nmea_parser.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

// Most recent date (ddmmyy) seen in an RMC sentence. GGA carries no date,
// so we cache it to build a full unix timestamp for either sentence type.
static int s_cached_ddmmyy = 0;

void nmea_parser_reset(void) {
    s_cached_ddmmyy = 0;
}

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

bool nmea_validate_checksum(const char *sentence) {
    if (sentence == NULL || sentence[0] != '$') return false;
    uint8_t checksum = 0;
    const char *p = sentence + 1; // skip '$'
    while (*p && *p != '*') {
        checksum ^= (uint8_t)*p++;
    }
    if (*p != '*') return false;          // no checksum delimiter
    int hi = hex_nibble(p[1]);
    int lo = hex_nibble(p[2]);
    if (hi < 0 || lo < 0) return false;   // malformed hex
    uint8_t expected = (uint8_t)((hi << 4) | lo);
    return checksum == expected;
}

// Copy comma-delimited fields of the sentence body (between '$' and '*')
// into a flat buffer, returning pointers into it. Returns field count.
#define NMEA_MAX_FIELDS 24
#define NMEA_BODY_MAX   120

static int split_fields(const char *sentence, char *buf, size_t buf_len,
                        char *fields[NMEA_MAX_FIELDS]) {
    const char *start = sentence + 1; // skip '$'
    const char *end = strchr(start, '*');
    size_t body_len = end ? (size_t)(end - start) : strlen(start);
    if (body_len >= buf_len) return -1;

    memcpy(buf, start, body_len);
    buf[body_len] = '\0';

    int count = 0;
    char *tok = buf;
    fields[count++] = tok;
    for (size_t i = 0; i < body_len && count < NMEA_MAX_FIELDS; i++) {
        if (buf[i] == ',') {
            buf[i] = '\0';
            fields[count++] = &buf[i + 1];
        }
    }
    return count;
}

// NMEA latitude is ddmm.mmmm, longitude is dddmm.mmmm. Convert to decimal deg.
static bool parse_lat(const char *val, const char *hemi, double *out) {
    if (val[0] == '\0') return false;
    double raw = atof(val);
    double deg = floor(raw / 100.0);
    double min = raw - deg * 100.0;
    double dec = deg + min / 60.0;
    if (hemi[0] == 'S') dec = -dec;
    *out = dec;
    return true;
}

static bool parse_lon(const char *val, const char *hemi, double *out) {
    if (val[0] == '\0') return false;
    double raw = atof(val);
    double deg = floor(raw / 100.0);
    double min = raw - deg * 100.0;
    double dec = deg + min / 60.0;
    if (hemi[0] == 'W') dec = -dec;
    *out = dec;
    return true;
}

// hhmmss.ss -> milliseconds since midnight (UTC).
static int64_t parse_time_ms(const char *val) {
    if (strlen(val) < 6) return 0;
    int hh = (val[0] - '0') * 10 + (val[1] - '0');
    int mm = (val[2] - '0') * 10 + (val[3] - '0');
    double ss = atof(val + 4);
    return (int64_t)((hh * 3600 + mm * 60) * 1000) + (int64_t)(ss * 1000.0);
}

// Days from civil date to unix epoch (Howard Hinnant's algorithm).
static int64_t days_from_civil(int y, int m, int d) {
    y -= (m <= 2);
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    int yoe = (int)(y - era * 400);
    int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}

static int64_t build_timestamp_ms(const char *time_field) {
    int64_t ms_of_day = parse_time_ms(time_field);
    if (s_cached_ddmmyy <= 0) {
        return ms_of_day; // no date yet — caller treats as ms-of-day
    }
    int dd = s_cached_ddmmyy / 10000;
    int mm = (s_cached_ddmmyy / 100) % 100;
    int yy = s_cached_ddmmyy % 100;
    int year = 2000 + yy;
    int64_t days = days_from_civil(year, mm, dd);
    return days * 86400000LL + ms_of_day;
}

static NmeaParseResult_t parse_gga(char *fields[], int n, NmeaFix_t *out) {
    if (n < 10) return NMEA_PARSE_INCOMPLETE;
    // fields: 0=type 1=time 2=lat 3=N/S 4=lon 5=E/W 6=quality
    //         7=numSats 8=hdop 9=alt ...
    out->fix_quality = (uint8_t)atoi(fields[6]);
    if (!parse_lat(fields[2], fields[3], &out->lat)) { out->valid = false; }
    parse_lon(fields[4], fields[5], &out->lon);
    out->num_sats = (uint8_t)atoi(fields[7]);
    out->hdop = (float)atof(fields[8]);
    out->alt_m = (float)atof(fields[9]);
    out->timestamp_ms = build_timestamp_ms(fields[1]);
    out->valid = (out->fix_quality > 0 && fields[2][0] != '\0');
    return NMEA_PARSE_OK;
}

static NmeaParseResult_t parse_rmc(char *fields[], int n, NmeaFix_t *out) {
    if (n < 10) return NMEA_PARSE_INCOMPLETE;
    // fields: 0=type 1=time 2=status 3=lat 4=N/S 5=lon 6=E/W
    //         7=speed(knots) 8=heading 9=date(ddmmyy)
    bool active = (fields[2][0] == 'A');
    if (fields[9][0] != '\0') {
        s_cached_ddmmyy = atoi(fields[9]);
    }
    parse_lat(fields[3], fields[4], &out->lat);
    parse_lon(fields[5], fields[6], &out->lon);
    out->speed_mps = (float)(atof(fields[7]) * 0.514444); // knots -> m/s
    out->heading_deg = (float)atof(fields[8]);
    out->timestamp_ms = build_timestamp_ms(fields[1]);
    out->valid = active && fields[3][0] != '\0';
    return NMEA_PARSE_OK;
}

NmeaParseResult_t nmea_parse_sentence(const char *sentence, NmeaFix_t *out) {
    if (sentence == NULL || out == NULL) return NMEA_PARSE_INCOMPLETE;
    if (sentence[0] != '$') return NMEA_PARSE_INCOMPLETE;
    if (!nmea_validate_checksum(sentence)) return NMEA_PARSE_INVALID_CHECKSUM;

    char buf[NMEA_BODY_MAX];
    char *fields[NMEA_MAX_FIELDS];
    int n = split_fields(sentence, buf, sizeof(buf), fields);
    if (n < 1) return NMEA_PARSE_INCOMPLETE;

    // Talker is first 2 chars (GN/GP/GL/GA); sentence type is last 3.
    const char *type = fields[0];
    size_t len = strlen(type);
    if (len < 5) return NMEA_PARSE_UNKNOWN_SENTENCE;
    const char *kind = type + (len - 3);

    if (strcmp(kind, "GGA") == 0) return parse_gga(fields, n, out);
    if (strcmp(kind, "RMC") == 0) return parse_rmc(fields, n, out);
    return NMEA_PARSE_UNKNOWN_SENTENCE;
}
