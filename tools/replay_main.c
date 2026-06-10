// Desktop replay harness: feed a recorded .nmea file through the full pipeline
// (parser -> EKF -> spoofing detector) and emit CSV for plotting.
//
//   ./navguard_replay drive.nmea > out.csv
//   python3 tools/plot_ekf.py out.csv
//
// CSV columns: ts_ms,gps_lat,gps_lon,ekf_lat,ekf_lon,state,gps_weight
#include <stdio.h>
#include <string.h>

#include "ekf.h"
#include "nmea_parser.h"
#include "spoof_detect.h"

static const char *state_name(GpsState_e s) {
    switch (s) {
    case GPS_LOCK:    return "LOCK";
    case GPS_SUSPECT: return "SUSPECT";
    case GPS_SPOOFED: return "SPOOFED";
    default:          return "DR_ONLY";
    }
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s <file.nmea>\n", argv[0]);
        return 2;
    }
    FILE *f = fopen(argv[1], "r");
    if (f == NULL) {
        fprintf(stderr, "cannot open %s\n", argv[1]);
        return 1;
    }

    EkfContext_t ekf;
    SpoofDetector_t sd;
    ekf_init(&ekf);
    spoof_init(&sd);
    nmea_parser_reset();

    printf("ts_ms,gps_lat,gps_lon,ekf_lat,ekf_lon,state,gps_weight\n");

    NmeaFix_t accum;
    memset(&accum, 0, sizeof(accum));
    ImuData_t imu;
    memset(&imu, 0, sizeof(imu));

    char line[160];
    while (fgets(line, sizeof(line), f) != NULL) {
        line[strcspn(line, "\r\n")] = '\0';
        if (line[0] != '$') continue;
        if (nmea_parse_sentence(line, &accum) != NMEA_PARSE_OK) continue;
        if (strstr(line, "GGA") == NULL || !accum.valid) continue;

        // Coast a few predict steps between fixes (no real IMU in replay).
        for (int i = 0; i < 40; i++) ekf_predict(&ekf, &imu, 0.0025f);

        GpsState_e st = spoof_update(&sd, &ekf, &accum, NULL);
        ekf_update_gps(&ekf, &accum);

        NmeaFix_t est;
        ekf_get_position(&ekf, &est);
        printf("%lld,%.7f,%.7f,%.7f,%.7f,%s,%.2f\n",
               (long long)accum.timestamp_ms, accum.lat, accum.lon,
               est.lat, est.lon, state_name(st), ekf.gps_weight);
    }
    fclose(f);
    return 0;
}
