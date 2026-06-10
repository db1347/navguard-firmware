#include "spoof_detect.h"

#include <math.h>
#include <string.h>
#include "geo_math.h"

void spoof_init(SpoofDetector_t *sd) {
    memset(sd, 0, sizeof(*sd));
    sd->state = GPS_LOCK; // optimistic until first fix proves otherwise
    sd->clean_streak = 0;
    sd->spoofed_since_ms = 0;
    sd->consistent_since_ms = 0;
}

bool spoof_velocity_gate_pass(const EkfContext_t *ctx, const NmeaFix_t *fix) {
    if (!ctx->initialized) return true; // nothing to compare against yet
    double dist_m = haversine_distance_m(ctx->state.lat, ctx->state.lon,
                                         fix->lat, fix->lon);
    float dt_s = (fix->timestamp_ms - ctx->last_gps_ms) / 1000.0f;
    if (dt_s <= 0.0f || dt_s > 5.0f) return false; // stale/garbled timing
    float implied_speed = (float)dist_m / dt_s;

    // Innovation gate: the position jump may exceed the EKF's expected travel
    // by up to VELOCITY_GATE_MPS (accel + GPS noise headroom), but a car cannot
    // move hundreds of m/s. This catches the "suddenly in Beirut" event in one
    // sample while tolerating normal highway speeds.
    float expected_speed = sqrtf(
        ctx->state.vel_north_mps * ctx->state.vel_north_mps +
        ctx->state.vel_east_mps * ctx->state.vel_east_mps);
    return implied_speed < (expected_speed + VELOCITY_GATE_MPS);
}

bool spoof_heading_check_pass(const EkfContext_t *ctx, const NmeaFix_t *fix) {
    if (!ctx->initialized) return true;
    // Compare how much GPS heading changed vs how much the IMU-tracked
    // heading changed over the same interval.
    float gps_delta = heading_diff_deg(fix->heading_deg, ctx->last_gps_heading);
    float imu_delta = heading_diff_deg(ctx->state.heading_deg,
                                       ctx->last_ekf_heading);
    float diff = fabsf(gps_delta - imu_delta);
    if (diff > 180.0f) diff = 360.0f - diff;
    return diff < HEADING_TOLERANCE_DEG;
}

bool spoof_cell_anchor_pass(const NmeaFix_t *fix, const CellFix_t *cell) {
    (void)fix;
    if (cell == NULL || !cell->valid) return true; // no anchor => can't refute
    // V1: country-level check only. The serving tower being in Israel while
    // GPS claims a neighbouring country is definitive spoofing.
    return cell->mcc == ISRAEL_MCC;
}

void spoof_on_gps_lost(SpoofDetector_t *sd, EkfContext_t *ctx) {
    sd->state = DR_ONLY;
    sd->clean_streak = 0;
    ekf_set_gps_state(ctx, DR_ONLY, WEIGHT_DR_ONLY);
}

GpsState_e spoof_update(SpoofDetector_t *sd, EkfContext_t *ctx,
                        const NmeaFix_t *fix, const CellFix_t *cell) {
    bool vel_ok = spoof_velocity_gate_pass(ctx, fix);
    bool hdg_ok = spoof_heading_check_pass(ctx, fix);
    bool cell_ok = spoof_cell_anchor_pass(fix, cell);
    sd->last_velocity_pass = vel_ok;
    sd->last_heading_pass = hdg_ok;
    sd->last_cell_pass = cell_ok;

    bool all_pass = vel_ok && hdg_ok && cell_ok;

    // Cell anchor contradiction is definitive: jump straight to SPOOFED.
    if (!cell_ok) {
        if (sd->state != GPS_SPOOFED) sd->spoofed_since_ms = fix->timestamp_ms;
        sd->state = GPS_SPOOFED;
        sd->clean_streak = 0;
    } else {
        switch (sd->state) {
        case GPS_LOCK:
            if (!all_pass) {
                sd->state = GPS_SUSPECT;
                sd->clean_streak = 0;
            }
            break;

        case GPS_SUSPECT:
            if (all_pass) {
                if (++sd->clean_streak >= SUSPECT_RECOVER_COUNT) {
                    sd->state = GPS_LOCK;
                    sd->clean_streak = 0;
                }
            } else {
                sd->clean_streak = 0;
                // A hard physical violation (teleport) confirms spoofing.
                if (!vel_ok) {
                    sd->state = GPS_SPOOFED;
                    sd->spoofed_since_ms = fix->timestamp_ms;
                }
            }
            break;

        case GPS_SPOOFED:
            if (all_pass) {
                if (sd->consistent_since_ms == 0) {
                    sd->consistent_since_ms = fix->timestamp_ms;
                } else if ((fix->timestamp_ms - sd->consistent_since_ms) / 1000.0f
                           >= SPOOFED_RECOVER_S) {
                    sd->state = GPS_LOCK;
                    sd->consistent_since_ms = 0;
                }
            } else {
                sd->consistent_since_ms = 0;
            }
            break;

        case DR_ONLY:
            // GPS returned. Treat the first fix as suspect until validated.
            sd->state = all_pass ? GPS_LOCK : GPS_SUSPECT;
            sd->clean_streak = 0;
            break;
        }
    }

    float weight;
    switch (sd->state) {
    case GPS_LOCK:    weight = WEIGHT_LOCK;    break;
    case GPS_SUSPECT: weight = WEIGHT_SUSPECT; break;
    case GPS_SPOOFED: weight = WEIGHT_SPOOFED; break;
    default:          weight = WEIGHT_DR_ONLY; break;
    }
    ekf_set_gps_state(ctx, sd->state, weight);
    return sd->state;
}
