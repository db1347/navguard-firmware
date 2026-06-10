// Spoofing state machine tests, including the "Beirut jump" acceptance test:
// state must reach GPS_SPOOFED within 3 EKF update cycles (<400ms).
#include "spoof_detect.h"
#include "ekf.h"

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

static NmeaFix_t fix_at(double lat, double lon, float speed_mps,
                        float heading_deg, int64_t ts_ms) {
    NmeaFix_t f;
    memset(&f, 0, sizeof(f));
    f.lat = lat;
    f.lon = lon;
    f.alt_m = 30.0f;
    f.speed_mps = speed_mps;
    f.heading_deg = heading_deg;
    f.fix_quality = 1;
    f.num_sats = 12;
    f.hdop = 0.8f;
    f.timestamp_ms = ts_ms;
    f.valid = true;
    return f;
}

// Drive normally for `n` fixes at 10Hz heading east; returns last timestamp.
static int64_t warmup_drive(SpoofDetector_t *sd, EkfContext_t *ekf, int n) {
    double lat = 32.0853, lon = 34.7818;
    int64_t ts = 0;
    for (int i = 0; i < n; i++) {
        ts += 100;
        lon += (13.4 * 0.1) / (111111.0 * cos(lat * M_PI / 180.0));
        NmeaFix_t f = fix_at(lat, lon, 13.4f, 90.0f, ts);
        spoof_update(sd, ekf, &f, NULL);
        ekf_update_gps(ekf, &f);
        // keep EKF heading tracking via predict (straight line, no rotation)
        ImuData_t imu = {0};
        for (int k = 0; k < 40; k++) ekf_predict(ekf, &imu, 0.0025f);
    }
    return ts;
}

static void test_normal_drive_stays_locked(void) {
    SpoofDetector_t sd;
    EkfContext_t ekf;
    spoof_init(&sd);
    ekf_init(&ekf);
    warmup_drive(&sd, &ekf, 100); // 10 seconds
    CHECK(sd.state == GPS_LOCK);
}

static void test_beirut_jump_detected_fast(void) {
    SpoofDetector_t sd;
    EkfContext_t ekf;
    spoof_init(&sd);
    ekf_init(&ekf);
    int64_t ts = warmup_drive(&sd, &ekf, 100);
    CHECK(sd.state == GPS_LOCK);

    // Spoofer teleports the position to Beirut. Count updates to SPOOFED.
    int updates_needed = 0;
    for (int i = 1; i <= 5; i++) {
        ts += 100;
        NmeaFix_t beirut = fix_at(33.8938, 35.5018, 13.4f, 90.0f, ts);
        GpsState_e s = spoof_update(&sd, &ekf, &beirut, NULL);
        ekf_update_gps(&ekf, &beirut); // weight is now 0.2 then 0.0
        updates_needed = i;
        if (s == GPS_SPOOFED) break;
    }
    printf("  Beirut jump -> GPS_SPOOFED in %d updates (%d ms)\n",
           updates_needed, updates_needed * 100);
    CHECK(sd.state == GPS_SPOOFED);
    CHECK(updates_needed <= 3); // < 400ms at 10Hz

    // EKF must not have moved to Beirut.
    NmeaFix_t est;
    ekf_get_position(&ekf, &est);
    CHECK(est.lat < 33.0); // still in Israel
}

static void test_city_driving_noise_passes(void) {
    SpoofDetector_t sd;
    EkfContext_t ekf;
    spoof_init(&sd);
    ekf_init(&ekf);
    double lat = 32.0853, lon = 34.7818;
    int64_t ts = 0;
    // 10 seconds of driving east with sub-metre per-sample multipath jitter —
    // the normal urban GPS picture. Must stay LOCK throughout.
    for (int i = 0; i < 100; i++) {
        ts += 100;
        lon += (13.4 * 0.1) / (111111.0 * cos(lat * M_PI / 180.0));
        double jitter_lat = ((i % 2) ? 0.6 : -0.6) / 111111.0; // +/-0.6 m
        NmeaFix_t f = fix_at(lat + jitter_lat, lon, 13.4f, 90.0f, ts);
        GpsState_e s = spoof_update(&sd, &ekf, &f, NULL);
        ekf_update_gps(&ekf, &f);
        ImuData_t imu = {0};
        for (int k = 0; k < 40; k++) ekf_predict(&ekf, &imu, 0.0025f);
        CHECK(s == GPS_LOCK);
    }
}

static void test_cell_anchor_contradiction_definitive(void) {
    SpoofDetector_t sd;
    EkfContext_t ekf;
    spoof_init(&sd);
    ekf_init(&ekf);
    int64_t ts = warmup_drive(&sd, &ekf, 50);

    // GPS looks locally consistent, but the serving cell is not Israeli MCC.
    // (e.g. replayed signal). One contradicting cell scan => SPOOFED.
    NmeaFix_t est;
    ekf_get_position(&ekf, &est);
    ts += 100;
    NmeaFix_t f = fix_at(est.lat, est.lon + 1.2 / 111111.0, 13.4f, 90.0f, ts);
    CellFix_t cell = { .mcc = 415, .mnc = 1, .lac = 0x2B4A, .cid = 1234,
                       .rssi = -85.0f, .valid = true }; // 415 = Lebanon
    GpsState_e s = spoof_update(&sd, &ekf, &f, &cell);
    CHECK(s == GPS_SPOOFED);
}

static void test_israeli_cell_passes(void) {
    SpoofDetector_t sd;
    EkfContext_t ekf;
    spoof_init(&sd);
    ekf_init(&ekf);
    int64_t ts = warmup_drive(&sd, &ekf, 50);

    NmeaFix_t est;
    ekf_get_position(&ekf, &est);
    ts += 100;
    NmeaFix_t f = fix_at(est.lat, est.lon + 1.2 / 111111.0, 13.4f, 90.0f, ts);
    CellFix_t cell = { .mcc = 425, .mnc = 1, .lac = 0x2B4A, .cid = 1234,
                       .rssi = -85.0f, .valid = true }; // HOT Mobile, Israel
    GpsState_e s = spoof_update(&sd, &ekf, &f, &cell);
    CHECK(s == GPS_LOCK);
}

static void test_suspect_recovers_after_clean_streak(void) {
    SpoofDetector_t sd;
    EkfContext_t ekf;
    spoof_init(&sd);
    ekf_init(&ekf);
    int64_t ts = warmup_drive(&sd, &ekf, 50);

    // One position jump beyond the innovation gate but far short of a teleport
    // (~5 m lateral in 100ms) -> SUSPECT, not SPOOFED.
    NmeaFix_t est;
    ekf_get_position(&ekf, &est);
    double lat = est.lat, lon = est.lon;
    ts += 100;
    NmeaFix_t jumpy = fix_at(lat + 5.0 / 111111.0, lon, 13.4f, 90.0f, ts);
    spoof_update(&sd, &ekf, &jumpy, NULL);
    CHECK(sd.state == GPS_SUSPECT);
    ekf_update_gps(&ekf, &jumpy); // weight 0.2: EKF barely follows the glitch

    // The glitch passes; GPS snaps back to the true track (where the EKF still
    // is). 3 clean consistent fixes -> back to LOCK.
    for (int i = 0; i < SUSPECT_RECOVER_COUNT; i++) {
        ts += 100;
        lon += (13.4 * 0.1) / (111111.0 * cos(lat * M_PI / 180.0));
        NmeaFix_t f = fix_at(lat, lon, 13.4f, 90.0f, ts);
        spoof_update(&sd, &ekf, &f, NULL);
        ekf_update_gps(&ekf, &f);
        ImuData_t imu = {0};
        for (int k = 0; k < 40; k++) ekf_predict(&ekf, &imu, 0.0025f);
    }
    CHECK(sd.state == GPS_LOCK);
}

static void test_gps_lost_enters_dr_only(void) {
    SpoofDetector_t sd;
    EkfContext_t ekf;
    spoof_init(&sd);
    ekf_init(&ekf);
    warmup_drive(&sd, &ekf, 50);

    spoof_on_gps_lost(&sd, &ekf); // tunnel entrance
    CHECK(sd.state == DR_ONLY);
    CHECK(ekf.gps_weight == 0.0f);
}

int main(void) {
    printf("test_spoof_detect\n");
    test_normal_drive_stays_locked();
    test_beirut_jump_detected_fast();
    test_city_driving_noise_passes();
    test_cell_anchor_contradiction_definitive();
    test_israeli_cell_passes();
    test_suspect_recovers_after_clean_streak();
    test_gps_lost_enters_dr_only();
    if (g_failures == 0) {
        printf("  all passed\n");
        return 0;
    }
    printf("  %d failure(s)\n", g_failures);
    return 1;
}
