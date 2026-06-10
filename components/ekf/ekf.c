#include "ekf.h"

#include <math.h>
#include <string.h>
#include "geo_math.h"

// Process / measurement noise tuning. These are scalar approximations of a
// full covariance treatment — sufficient for the V1 fusion behaviour the
// dev plan specifies (GPS-weighted blend + dead reckoning + ZUPT).
#define EKF_POS_PROCESS_VAR    0.05f   // m^2 added per second of DR
#define EKF_VEL_PROCESS_VAR    0.10f
#define EKF_HDG_PROCESS_VAR    0.20f
#define EKF_ZUPT_SPEED_MPS     0.15f
#define EKF_MAX_POS_VAR        2500.0f // clamp uncertainty growth

void ekf_init(EkfContext_t *ctx) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->gps_state = DR_ONLY;
    ctx->gps_weight = 0.0f;
    ctx->initialized = false;
    // Seed diagonal covariance with high uncertainty until first GPS fix.
    for (int i = 0; i < 7; i++) ctx->P[i][i] = 1000.0f;
}

void ekf_set_gps_state(EkfContext_t *ctx, GpsState_e state, float weight) {
    ctx->gps_state = state;
    ctx->gps_weight = weight;
}

static void propagate_covariance(EkfContext_t *ctx, float dt_s) {
    ctx->P[0][0] += EKF_POS_PROCESS_VAR * dt_s; // lat
    ctx->P[1][1] += EKF_POS_PROCESS_VAR * dt_s; // lon
    ctx->P[2][2] += EKF_POS_PROCESS_VAR * dt_s; // alt
    ctx->P[3][3] += EKF_VEL_PROCESS_VAR * dt_s; // vN
    ctx->P[4][4] += EKF_VEL_PROCESS_VAR * dt_s; // vE
    ctx->P[5][5] += EKF_VEL_PROCESS_VAR * dt_s; // vD
    ctx->P[6][6] += EKF_HDG_PROCESS_VAR * dt_s; // heading
    for (int i = 0; i < 7; i++) {
        if (ctx->P[i][i] > EKF_MAX_POS_VAR) ctx->P[i][i] = EKF_MAX_POS_VAR;
    }
}

void ekf_predict(EkfContext_t *ctx, const ImuData_t *imu, float dt_s) {
    if (dt_s <= 0.0f) return;
    if (!ctx->initialized) return; // need an absolute anchor first

    // Integrate heading from gyro yaw rate.
    ctx->state.heading_deg = wrap_heading_deg(
        ctx->state.heading_deg + imu->gyro_z_dps * dt_s);

    // Rotate body-frame acceleration to NED (north/east) frame.
    float hdg_rad = ctx->state.heading_deg * (float)M_PI / 180.0f;
    float accel_north = imu->accel_x_mss * cosf(hdg_rad) -
                        imu->accel_y_mss * sinf(hdg_rad);
    float accel_east  = imu->accel_x_mss * sinf(hdg_rad) +
                        imu->accel_y_mss * cosf(hdg_rad);

    ctx->state.vel_north_mps += accel_north * dt_s;
    ctx->state.vel_east_mps  += accel_east  * dt_s;

    // Zero-velocity update: kill integrated drift when stationary.
    float speed = sqrtf(ctx->state.vel_north_mps * ctx->state.vel_north_mps +
                        ctx->state.vel_east_mps * ctx->state.vel_east_mps);
    if (speed < EKF_ZUPT_SPEED_MPS && imu->is_stationary) {
        ctx->state.vel_north_mps = 0.0f;
        ctx->state.vel_east_mps  = 0.0f;
    }

    // Integrate position (m/s -> degrees).
    ctx->state.lat += (ctx->state.vel_north_mps * dt_s) / NG_METERS_PER_DEG_LAT;
    double cos_lat = cos(ctx->state.lat * M_PI / 180.0);
    if (fabs(cos_lat) > 1e-6) {
        ctx->state.lon += (ctx->state.vel_east_mps * dt_s) /
                          (NG_METERS_PER_DEG_LAT * cos_lat);
    }

    propagate_covariance(ctx, dt_s);
    ctx->last_ekf_heading = ctx->state.heading_deg;
}

void ekf_update_gps(EkfContext_t *ctx, const NmeaFix_t *fix) {
    if (fix == NULL || !fix->valid) return;

    // First valid fix anchors the filter regardless of weight.
    if (!ctx->initialized) {
        ctx->state.lat = fix->lat;
        ctx->state.lon = fix->lon;
        ctx->state.alt_m = fix->alt_m;
        ctx->state.heading_deg = fix->heading_deg;
        float vN = fix->speed_mps * cosf(fix->heading_deg * (float)M_PI / 180.0f);
        float vE = fix->speed_mps * sinf(fix->heading_deg * (float)M_PI / 180.0f);
        ctx->state.vel_north_mps = vN;
        ctx->state.vel_east_mps = vE;
        ctx->initialized = true;
        ctx->last_gps_ms = fix->timestamp_ms;
        ctx->last_gps_heading = fix->heading_deg;
        for (int i = 0; i < 7; i++) ctx->P[i][i] = 9.0f; // ~3m std
        return;
    }

    float w = ctx->gps_weight;
    if (w <= 0.0f) {
        // Spoofed / DR-only: ignore GPS position entirely.
        ctx->last_gps_ms = fix->timestamp_ms;
        ctx->last_gps_heading = fix->heading_deg;
        return;
    }
    if (w > 1.0f) w = 1.0f;

    // Weighted blend toward the GPS observation (scalar gain per state).
    ctx->state.lat += (fix->lat - ctx->state.lat) * w;
    ctx->state.lon += (fix->lon - ctx->state.lon) * w;
    ctx->state.alt_m += (fix->alt_m - ctx->state.alt_m) * w;

    if (fix->speed_mps > 0.5f) {
        float vN = fix->speed_mps * cosf(fix->heading_deg * (float)M_PI / 180.0f);
        float vE = fix->speed_mps * sinf(fix->heading_deg * (float)M_PI / 180.0f);
        ctx->state.vel_north_mps += (vN - ctx->state.vel_north_mps) * w;
        ctx->state.vel_east_mps  += (vE - ctx->state.vel_east_mps) * w;
        // Trust GPS heading when actually moving.
        ctx->state.heading_deg = wrap_heading_deg(fix->heading_deg);
    }

    // Correcting shrinks uncertainty proportional to the gain.
    for (int i = 0; i < 7; i++) ctx->P[i][i] *= (1.0f - w);

    ctx->last_gps_ms = fix->timestamp_ms;
    ctx->last_gps_heading = fix->heading_deg;
}

void ekf_update_cell(EkfContext_t *ctx, const CellFix_t *cell) {
    // V1: the cell fix is consumed by the spoofing detector as a country-level
    // anchor, not as a position measurement. Keep the latest reading only.
    (void)ctx;
    (void)cell;
}

void ekf_update_baro(EkfContext_t *ctx, float alt_m) {
    if (!ctx->initialized) return;
    const float w = 0.3f; // barometer is steady but biased; modest weight
    ctx->state.alt_m += (alt_m - ctx->state.alt_m) * w;
}

void ekf_get_position(const EkfContext_t *ctx, NmeaFix_t *out) {
    memset(out, 0, sizeof(*out));
    out->lat = ctx->state.lat;
    out->lon = ctx->state.lon;
    out->alt_m = ctx->state.alt_m;
    float speed = sqrtf(ctx->state.vel_north_mps * ctx->state.vel_north_mps +
                        ctx->state.vel_east_mps * ctx->state.vel_east_mps);
    out->speed_mps = speed;
    out->heading_deg = ctx->state.heading_deg;
    out->fix_quality = (ctx->gps_state == GPS_LOCK) ? 1 : 0;
    out->num_sats = 12;
    out->hdop = (ctx->gps_state == GPS_LOCK) ? 0.8f : 5.0f;
    out->timestamp_ms = ctx->last_gps_ms;
    out->valid = ctx->initialized;
}
