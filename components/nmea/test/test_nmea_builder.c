// Round-trip tests: build NMEA from a fix, parse it back, compare.
#include "nmea_builder.h"
#include "nmea_parser.h"

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

static void test_gga_roundtrip(void) {
    NmeaFix_t src;
    memset(&src, 0, sizeof(src));
    src.lat = 32.0805123;
    src.lon = 34.7805987;
    src.alt_m = 42.5f;
    src.fix_quality = 1;
    src.num_sats = 11;
    src.hdop = 0.9f;
    src.timestamp_ms = 43200000; // 12:00:00 ms-of-day

    char sentence[96];
    int len = nmea_build_gngga(&src, sentence, sizeof(sentence));
    CHECK(len > 0);
    CHECK(nmea_validate_checksum(sentence));

    NmeaFix_t back;
    memset(&back, 0, sizeof(back));
    CHECK(nmea_parse_sentence(sentence, &back) == NMEA_PARSE_OK);
    CHECK(fabs(back.lat - src.lat) < 1e-5);
    CHECK(fabs(back.lon - src.lon) < 1e-5);
    CHECK(fabs(back.alt_m - src.alt_m) < 0.2);
    CHECK(back.fix_quality == src.fix_quality);
    CHECK(back.num_sats == src.num_sats);
}

static void test_rmc_roundtrip(void) {
    NmeaFix_t src;
    memset(&src, 0, sizeof(src));
    src.lat = 32.0805123;
    src.lon = 34.7805987;
    src.speed_mps = 13.4f;
    src.heading_deg = 187.6f;
    src.valid = true;
    src.timestamp_ms = 43200000;

    char sentence[96];
    int len = nmea_build_gnrmc(&src, sentence, sizeof(sentence));
    CHECK(len > 0);
    CHECK(nmea_validate_checksum(sentence));

    NmeaFix_t back;
    memset(&back, 0, sizeof(back));
    CHECK(nmea_parse_sentence(sentence, &back) == NMEA_PARSE_OK);
    CHECK(fabs(back.lat - src.lat) < 1e-5);
    CHECK(fabs(back.lon - src.lon) < 1e-5);
    CHECK(fabs(back.speed_mps - src.speed_mps) < 0.1);
    CHECK(fabs(back.heading_deg - src.heading_deg) < 0.2);
}

static void test_southern_hemisphere_roundtrip(void) {
    NmeaFix_t src;
    memset(&src, 0, sizeof(src));
    src.lat = -33.8688;
    src.lon = -70.1234; // western too
    src.fix_quality = 1;
    src.num_sats = 9;
    src.hdop = 1.1f;

    char sentence[96];
    CHECK(nmea_build_gngga(&src, sentence, sizeof(sentence)) > 0);
    NmeaFix_t back;
    memset(&back, 0, sizeof(back));
    CHECK(nmea_parse_sentence(sentence, &back) == NMEA_PARSE_OK);
    CHECK(fabs(back.lat - src.lat) < 1e-5);
    CHECK(fabs(back.lon - src.lon) < 1e-5);
    CHECK(back.lat < 0.0 && back.lon < 0.0);
}

static void test_buffer_too_small(void) {
    NmeaFix_t src;
    memset(&src, 0, sizeof(src));
    src.fix_quality = 1;
    char tiny[8];
    CHECK(nmea_build_gngga(&src, tiny, sizeof(tiny)) == -1);
}

int main(void) {
    printf("test_nmea_builder\n");
    test_gga_roundtrip();
    test_rmc_roundtrip();
    test_southern_hemisphere_roundtrip();
    test_buffer_too_small();
    if (g_failures == 0) {
        printf("  all passed\n");
        return 0;
    }
    printf("  %d failure(s)\n", g_failures);
    return 1;
}
