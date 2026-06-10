#ifndef NAVGUARD_SPOOF_DETECT_H
#define NAVGUARD_SPOOF_DETECT_H

#include <stdbool.h>
#include "ekf.h"

#ifdef __cplusplus
extern "C" {
#endif

// 50 km/h worth of movement in one sample is physically impossible for a car.
#define VELOCITY_GATE_MPS   14.0f
#define HEADING_TOLERANCE_DEG 35.0f
#define ISRAEL_MCC          425u

// GPS weights per state (from dev plan Section 4.3).
#define WEIGHT_LOCK     0.85f
#define WEIGHT_SUSPECT  0.20f
#define WEIGHT_SPOOFED  0.00f
#define WEIGHT_DR_ONLY  0.00f

// Number of consecutive clean fixes required to recover from SUSPECT->LOCK.
#define SUSPECT_RECOVER_COUNT 3
// Seconds of consistent GPS required to recover from SPOOFED->LOCK.
#define SPOOFED_RECOVER_S     10.0f

typedef struct {
    GpsState_e state;
    int        clean_streak;       // consecutive passing fixes (SUSPECT recovery)
    int64_t    spoofed_since_ms;   // when spoofing was first confirmed
    int64_t    consistent_since_ms;// when GPS started agreeing again
    bool       last_velocity_pass;
    bool       last_heading_pass;
    bool       last_cell_pass;
} SpoofDetector_t;

void spoof_init(SpoofDetector_t *sd);

// Run after each GPS fix. Updates state, then writes the resulting trust
// weight into the EKF via ekf_set_gps_state(). `cell` may be NULL/invalid.
// Returns the new state.
GpsState_e spoof_update(SpoofDetector_t *sd, EkfContext_t *ctx,
                        const NmeaFix_t *fix, const CellFix_t *cell);

// Call when GPS is fully absent (no fix this cycle) — enters DR_ONLY.
void spoof_on_gps_lost(SpoofDetector_t *sd, EkfContext_t *ctx);

// Individual gates, exposed for unit testing.
bool spoof_velocity_gate_pass(const EkfContext_t *ctx, const NmeaFix_t *fix);
bool spoof_heading_check_pass(const EkfContext_t *ctx, const NmeaFix_t *fix);
bool spoof_cell_anchor_pass(const NmeaFix_t *fix, const CellFix_t *cell);

#ifdef __cplusplus
}
#endif

#endif // NAVGUARD_SPOOF_DETECT_H
