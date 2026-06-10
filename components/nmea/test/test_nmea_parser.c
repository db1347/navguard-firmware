// Desktop unit tests for the NMEA parser (CTest).
#include "nmea_parser.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_failures = 0;

#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) {                                                      \
            printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);        \
            g_failures++;                                                   \
        }                                                                   \
    } while (0)

static void test_gngga_basic(void) {
    const char *s = "$GNGGA,120000.00,3212.1234,N,03456.5678,E,1,12,0.8,35.2,M,17.0,M,,*46";
    NmeaFix_t fix;
    memset(&fix, 0, sizeof(fix));
    NmeaParseResult_t r = nmea_parse_sentence(s, &fix);
    CHECK(r == NMEA_PARSE_OK);
    CHECK(fabs(fix.lat - 32.20206) < 0.001);
    CHECK(fabs(fix.lon - 34.94280) < 0.001);
    CHECK(fix.fix_quality == 1);
    CHECK(fix.num_sats == 12);
    CHECK(fabs(fix.hdop - 0.8) < 0.01);
    CHECK(fabs(fix.alt_m - 35.2) < 0.01);
    CHECK(fix.valid);
}

static void test_gnrmc_basic(void) {
    const char *s = "$GNRMC,120000.00,A,3212.1234,N,03456.5678,E,0.5,275.3,100624,,,A*4A";
    NmeaFix_t fix;
    memset(&fix, 0, sizeof(fix));
    NmeaParseResult_t r = nmea_parse_sentence(s, &fix);
    CHECK(r == NMEA_PARSE_OK);
    CHECK(fabs(fix.lat - 32.20206) < 0.001);
    CHECK(fix.speed_mps > 0.0f);            // 0.5 knots
    CHECK(fabs(fix.heading_deg - 275.3) < 0.1);
    CHECK(fix.valid);
}

static void test_southern_western_hemisphere(void) {
    // Sydney-ish: south latitude, east longitude
    const char *s = "$GPGGA,235959.00,3352.0000,S,15112.0000,E,1,08,1.0,10.0,M,,,,*1A";
    NmeaFix_t fix;
    memset(&fix, 0, sizeof(fix));
    NmeaParseResult_t r = nmea_parse_sentence(s, &fix);
    CHECK(r == NMEA_PARSE_OK);
    CHECK(fix.lat < 0.0);                    // southern hemisphere
    CHECK(fix.lon > 0.0);
}

static void test_western_longitude(void) {
    const char *s = "$GPRMC,000000.00,A,4044.0000,N,07359.0000,W,10.0,90.0,010120,,,A*4A";
    NmeaFix_t fix;
    memset(&fix, 0, sizeof(fix));
    NmeaParseResult_t r = nmea_parse_sentence(s, &fix);
    CHECK(r == NMEA_PARSE_OK);
    CHECK(fix.lon < 0.0);                    // western hemisphere => negative
    CHECK(fix.lat > 0.0);
}

static void test_invalid_checksum(void) {
    const char *s = "$GNGGA,120000.00,3212.1234,N,03456.5678,E,1,12,0.8,35.2,M,17.0,M,,*00";
    NmeaFix_t fix;
    CHECK(nmea_parse_sentence(s, &fix) == NMEA_PARSE_INVALID_CHECKSUM);
    CHECK(!nmea_validate_checksum(s));
}

static void test_missing_checksum_delimiter(void) {
    const char *s = "$GNGGA,120000.00,3212.1234,N,03456.5678,E,1,12,0.8,35.2,M";
    CHECK(!nmea_validate_checksum(s));
}

static void test_malformed_hex_checksum(void) {
    const char *s = "$GNGGA,120000.00,3212.1234,N,03456.5678,E,1,12,0.8,35.2,M*ZZ";
    CHECK(!nmea_validate_checksum(s));
}

static void test_truncated_sentence(void) {
    // Valid checksum on a too-short GGA body => INCOMPLETE.
    char buf[64] = "$GNGGA,120000.00,3212.1234,N";
    // Recompute a correct checksum for this truncated body.
    uint8_t cs = 0;
    for (const char *p = buf + 1; *p; p++) cs ^= (uint8_t)*p;
    char full[80];
    snprintf(full, sizeof(full), "%s*%02X", buf, cs);
    NmeaFix_t fix;
    memset(&fix, 0, sizeof(fix));
    CHECK(nmea_parse_sentence(full, &fix) == NMEA_PARSE_INCOMPLETE);
}

static void test_unknown_sentence(void) {
    // Well-formed checksum but a sentence type we don't parse.
    char body[] = "$GNVTG,275.3,T,,M,0.5,N,0.9,K,A";
    uint8_t cs = 0;
    for (const char *p = body + 1; *p; p++) cs ^= (uint8_t)*p;
    char full[64];
    snprintf(full, sizeof(full), "%s*%02X", body, cs);
    NmeaFix_t fix;
    CHECK(nmea_parse_sentence(full, &fix) == NMEA_PARSE_UNKNOWN_SENTENCE);
}

static void test_no_fix_quality_zero(void) {
    char body[] = "$GNGGA,120000.00,,,,,0,00,99.9,,M,,M,,";
    uint8_t cs = 0;
    for (const char *p = body + 1; *p; p++) cs ^= (uint8_t)*p;
    char full[80];
    snprintf(full, sizeof(full), "%s*%02X", body, cs);
    NmeaFix_t fix;
    memset(&fix, 0, sizeof(fix));
    CHECK(nmea_parse_sentence(full, &fix) == NMEA_PARSE_OK);
    CHECK(!fix.valid);                       // quality 0 => not valid
}

static void test_null_inputs(void) {
    NmeaFix_t fix;
    CHECK(nmea_parse_sentence(NULL, &fix) == NMEA_PARSE_INCOMPLETE);
    CHECK(nmea_parse_sentence("$GNGGA*00", NULL) == NMEA_PARSE_INCOMPLETE);
    CHECK(!nmea_validate_checksum(NULL));
}

static void test_not_a_sentence(void) {
    NmeaFix_t fix;
    CHECK(nmea_parse_sentence("garbage line", &fix) == NMEA_PARSE_INCOMPLETE);
}

static void test_rmc_date_feeds_gga_timestamp(void) {
    nmea_parser_reset();
    // RMC supplies the date; a following GGA should produce a full unix ms.
    const char *rmc = "$GNRMC,120000.00,A,3212.1234,N,03456.5678,E,0.5,275.3,100624,,,A*4A";
    const char *gga = "$GNGGA,120001.00,3212.1234,N,03456.5678,E,1,12,0.8,35.2,M,17.0,M,,*47";
    NmeaFix_t fix;
    memset(&fix, 0, sizeof(fix));
    nmea_parse_sentence(rmc, &fix);
    nmea_parse_sentence(gga, &fix);
    // 10 June 2024 is ~1.7e12 ms since epoch.
    CHECK(fix.timestamp_ms > 1700000000000LL);
}

int main(void) {
    printf("test_nmea_parser\n");
    test_gngga_basic();
    test_gnrmc_basic();
    test_southern_western_hemisphere();
    test_western_longitude();
    test_invalid_checksum();
    test_missing_checksum_delimiter();
    test_malformed_hex_checksum();
    test_truncated_sentence();
    test_unknown_sentence();
    test_no_fix_quality_zero();
    test_null_inputs();
    test_not_a_sentence();
    test_rmc_date_feeds_gga_timestamp();
    if (g_failures == 0) {
        printf("  all passed\n");
        return 0;
    }
    printf("  %d failure(s)\n", g_failures);
    return 1;
}
