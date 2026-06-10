#ifndef NAVGUARD_EKF_H
#define NAVGUARD_EKF_H

#include <stdbool.h>
#include <stdint.h>
#include "nmea_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

// GPS trust state, driven by the spoofing detector. The EKF reads the
// resulting gps_weight when applying a GPS correction.
typedef enum {
    GPS_LOCK = 0,   // valid, all checks pass
    GPS_SUSPECT,    // one check failed, under evaluation
    GPS_SPOOFED,    // cell anchor contradicts GPS
    DR_ONLY,        // GPS absent (tunnel / full jam)
} GpsState_e;

// IMU sample in SI units (already scaled from raw counts by the driver).
typedef struct {
    float gyro_z_dps;     // yaw rate, deg/s
    float accel_x_mss;    // body-frame forward acceleration, m/s^2
    float accel_y_mss;    // body-frame lateral acceleration, m/s^2
    bool  is_stationary;  // driver/heuristic flag for ZUPT
} ImuData_t;

// Cell scan result used as a coarse geographic anchor.
typedef struct {
    uint32_t mcc;   // Mobile Country Code (425 = Israel)
    uint32_t mnc;
    uint32_t lac;
    uint32_t cid;
    float    rssi;
    bool     valid;
} CellFix_t;

typedef struct {
    double  lat;
    double  lon;
    float   alt_m;
    float   vel_north_mps;
    float   vel_east_mps;
    float   vel_down_mps;
    float   heading_deg;
} EkfState_t;

typedef struct {
    EkfState_t state;
    float      P[7][7];        // covariance (diagonal maintained here)
    GpsState_e gps_state;
    float      gps_weight;     // 0..1, applied to GPS corrections
    int64_t    last_gps_ms;
    int64_t    last_imu_ms;
    float      last_gps_heading;
    float      last_ekf_heading;
    bool       initialized;    // false until first GPS fix anchors position
} EkfContext_t;

void ekf_init(EkfContext_t *ctx);

// Dead-reckoning predict step. Call at IMU rate (~400Hz on hardware).
void ekf_predict(EkfContext_t *ctx, const ImuData_t *imu, float dt_s);

// GPS correction. Weight comes from ctx->gps_weight (set by spoof detector).
void ekf_update_gps(EkfContext_t *ctx, const NmeaFix_t *fix);

// Coarse cell anchor correction (~1Hz). No-op in V1 beyond bookkeeping.
void ekf_update_cell(EkfContext_t *ctx, const CellFix_t *cell);

// Barometric altitude correction (~5Hz).
void ekf_update_baro(EkfContext_t *ctx, float alt_m);

// Read the current best estimate as an NmeaFix for downstream NMEA output.
void ekf_get_position(const EkfContext_t *ctx, NmeaFix_t *out);

// Set GPS trust weight directly (called by the spoofing detector).
void ekf_set_gps_state(EkfContext_t *ctx, GpsState_e state, float weight);

#ifdef __cplusplus
}
#endif

#endif // NAVGUARD_EKF_H
