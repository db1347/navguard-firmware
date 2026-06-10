// EKF desktop tests: convergence to ground truth and ZUPT drift control.
#include "ekf.h"
#include "geo_math.h"
#include "nmea_builder.h"
#include "nmea_parser.h"
#include "spoof_detect.h"

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

// Build a valid GPS fix moving north-east at constant speed from a Tel Aviv
// start point, then round-trip it through the NMEA layer so the EKF sees the
// same data path it would on hardware.
static NmeaFix_t make_fix(double lat, double lon, float speed_mps,
                          float heading_deg, int64_t ts_ms) {
    NmeaFix_t src;
    memset(&src, 0, sizeof(src));
    src.lat = lat;
    src.lon = lon;
    src.alt_m = 30.0f;
    src.speed_mps = speed_mps;
    src.heading_deg = heading_deg;
    src.fix_quality = 1;
    src.num_sats = 12;
    src.hdop = 0.8f;
    src.timestamp_ms = ts_ms;
    src.valid = true;

    char gga[96], rmc[96];
    nmea_build_gngga(&src, gga, sizeof(gga));
    nmea_build_gnrmc(&src, rmc, sizeof(rmc));

    NmeaFix_t parsed;
    memset(&parsed, 0, sizeof(parsed));
    nmea_parse_sentence(rmc, &parsed); // speed + heading
    nmea_parse_sentence(gga, &parsed); // lat/lon/alt/quality
    return parsed;
}

static void test_convergence_60s(void) {
    EkfContext_t ekf;
    ekf_init(&ekf);
    ekf_set_gps_state(&ekf, GPS_LOCK, WEIGHT_LOCK);

    // Ground truth: start in central Tel Aviv, drive due east at 13.4 m/s.
    double lat0 = 32.0853, lon0 = 34.7818;
    float speed = 13.4f;     // ~48 km/h
    float heading = 90.0f;   // due east
    double gt_lat = lat0, gt_lon = lon0;

    // 10Hz GPS, 400Hz IMU predict between fixes (40 predicts per GPS step).
    ImuData_t imu;
    memset(&imu, 0, sizeof(imu));
    imu.gyro_z_dps = 0.0f;          // straight line
    imu.accel_x_mss = 0.0f;         // constant speed
    imu.is_stationary = false;

    int64_t ts = 0;
    double max_err = 0.0;
    for (int step = 0; step < 600; step++) { // 60s at 10Hz
        ts += 100;
        // advance ground truth east by speed * 0.1s
        double dlon = (speed * 0.1) / (NG_METERS_PER_DEG_LAT *
                                       cos(gt_lat * M_PI / 180.0));
        gt_lon += dlon;

        NmeaFix_t fix = make_fix(gt_lat, gt_lon, speed, heading, ts);

        // IMU predict at 400Hz between this fix and the previous one.
        for (int i = 0; i < 40; i++) {
            ekf_predict(&ekf, &imu, 0.0025f);
        }
        ekf_update_gps(&ekf, &fix);

        NmeaFix_t est;
        ekf_get_position(&ekf, &est);
        if (est.valid) {
            double err = haversine_distance_m(gt_lat, gt_lon, est.lat, est.lon);
            if (step > 20 && err > max_err) max_err = err; // after warmup
        }
    }
    printf("  convergence max error after warmup: %.2f m\n", max_err);
    CHECK(max_err < 8.0);
}

static void test_zupt_stationary_drift(void) {
    EkfContext_t ekf;
    ekf_init(&ekf);
    ekf_set_gps_state(&ekf, GPS_LOCK, WEIGHT_LOCK);

    double lat0 = 32.0853, lon0 = 34.7818;
    // Anchor the filter with one fix.
    NmeaFix_t fix = make_fix(lat0, lon0, 0.0f, 0.0f, 0);
    ekf_update_gps(&ekf, &fix);

    // 30s stationary with small accelerometer noise; ZUPT should hold position.
    ImuData_t imu;
    memset(&imu, 0, sizeof(imu));
    imu.is_stationary = true;
    imu.accel_x_mss = 0.02f; // sensor noise
    imu.accel_y_mss = -0.015f;

    for (int i = 0; i < 30 * 400; i++) {
        ekf_predict(&ekf, &imu, 0.0025f);
    }
    NmeaFix_t est;
    ekf_get_position(&ekf, &est);
    double drift = haversine_distance_m(lat0, lon0, est.lat, est.lon);
    printf("  ZUPT 30s drift: %.3f m\n", drift);
    CHECK(drift < 1.0);
}

static void test_spoofed_weight_ignores_gps(void) {
    EkfContext_t ekf;
    ekf_init(&ekf);
    ekf_set_gps_state(&ekf, GPS_LOCK, WEIGHT_LOCK);
    NmeaFix_t anchor = make_fix(32.0853, 34.7818, 5.0f, 90.0f, 0);
    ekf_update_gps(&ekf, &anchor);

    // Now mark spoofed and feed a Beirut fix — position must not jump.
    ekf_set_gps_state(&ekf, GPS_SPOOFED, 0.0f);
    NmeaFix_t beirut = make_fix(33.8938, 35.5018, 5.0f, 90.0f, 100);
    ekf_update_gps(&ekf, &beirut);

    NmeaFix_t est;
    ekf_get_position(&ekf, &est);
    double dist_to_beirut = haversine_distance_m(33.8938, 35.5018,
                                                 est.lat, est.lon);
    CHECK(dist_to_beirut > 100000.0); // still ~hundreds of km from Beirut
}

int main(void) {
    printf("test_ekf\n");
    test_convergence_60s();
    test_zupt_stationary_drift();
    test_spoofed_weight_ignores_gps();
    if (g_failures == 0) {
        printf("  all passed\n");
        return 0;
    }
    printf("  %d failure(s)\n", g_failures);
    return 1;
}
