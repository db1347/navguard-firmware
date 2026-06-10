#include "nmea_builder.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

// Convert decimal degrees to NMEA ddmm.mmmmmm (lat) / dddmm.mmmmmm (lon).
static void deg_to_nmea(double dec, int deg_width, char *out, size_t out_len,
                        char *hemi, char pos, char neg) {
    *hemi = (dec < 0.0) ? neg : pos;
    double a = fabs(dec);
    int degrees = (int)floor(a);
    double minutes = (a - degrees) * 60.0;
    // deg_width selects %02d (lat) vs %03d (lon)
    if (deg_width == 3) {
        snprintf(out, out_len, "%03d%09.6f", degrees, minutes);
    } else {
        snprintf(out, out_len, "%02d%09.6f", degrees, minutes);
    }
}

int nmea_append_checksum(char *sentence, size_t out_len) {
    if (sentence == NULL || sentence[0] != '$') return -1;
    uint8_t checksum = 0;
    const char *p = sentence + 1;
    while (*p && *p != '*') {
        checksum ^= (uint8_t)*p++;
    }
    size_t len = strlen(sentence);
    int n = snprintf(sentence + len, out_len - len, "*%02X\r\n", checksum);
    if (n < 0 || (size_t)(len + n) >= out_len) return -1;
    return (int)(len + n);
}

// hhmmss.ss from a timestamp expressed in ms-of-day (or full unix ms).
static void fmt_time(int64_t ts_ms, char *out, size_t out_len) {
    int64_t ms_of_day = ((ts_ms % 86400000LL) + 86400000LL) % 86400000LL;
    int hh = (int)(ms_of_day / 3600000);
    int mm = (int)((ms_of_day / 60000) % 60);
    double ss = (ms_of_day % 60000) / 1000.0;
    snprintf(out, out_len, "%02d%02d%05.2f", hh, mm, ss);
}

int nmea_build_gngga(const NmeaFix_t *fix, char *out, size_t out_len) {
    if (fix == NULL || out == NULL) return -1;
    char lat[16], lon[16], tbuf[12];
    char ns, ew;
    deg_to_nmea(fix->lat, 2, lat, sizeof(lat), &ns, 'N', 'S');
    deg_to_nmea(fix->lon, 3, lon, sizeof(lon), &ew, 'E', 'W');
    fmt_time(fix->timestamp_ms, tbuf, sizeof(tbuf));

    int n = snprintf(out, out_len,
        "$GNGGA,%s,%s,%c,%s,%c,%u,%02u,%.1f,%.1f,M,17.0,M,,",
        tbuf, lat, ns, lon, ew,
        fix->fix_quality, fix->num_sats, fix->hdop, (double)fix->alt_m);
    if (n < 0 || (size_t)n >= out_len) return -1;
    return nmea_append_checksum(out, out_len);
}

int nmea_build_gnrmc(const NmeaFix_t *fix, char *out, size_t out_len) {
    if (fix == NULL || out == NULL) return -1;
    char lat[16], lon[16], tbuf[12];
    char ns, ew;
    deg_to_nmea(fix->lat, 2, lat, sizeof(lat), &ns, 'N', 'S');
    deg_to_nmea(fix->lon, 3, lon, sizeof(lon), &ew, 'E', 'W');
    fmt_time(fix->timestamp_ms, tbuf, sizeof(tbuf));

    double knots = fix->speed_mps / 0.514444;
    // Derive ddmmyy from the timestamp when it carries a real date.
    char date[8] = "";
    if (fix->timestamp_ms >= 86400000LL) {
        time_t secs = (time_t)(fix->timestamp_ms / 1000);
        struct tm tmv;
        gmtime_r(&secs, &tmv);
        snprintf(date, sizeof(date), "%02d%02d%02d",
                 tmv.tm_mday, tmv.tm_mon + 1, (tmv.tm_year + 1900) % 100);
    }

    int n = snprintf(out, out_len,
        "$GNRMC,%s,%c,%s,%c,%s,%c,%.1f,%.1f,%s,,,A",
        tbuf, fix->valid ? 'A' : 'V', lat, ns, lon, ew,
        knots, (double)fix->heading_deg, date);
    if (n < 0 || (size_t)n >= out_len) return -1;
    return nmea_append_checksum(out, out_len);
}
