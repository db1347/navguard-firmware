#ifndef NAVGUARD_NMEA_PARSER_H
#define NAVGUARD_NMEA_PARSER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double  lat;           // decimal degrees, negative = South
    double  lon;           // decimal degrees, negative = West
    float   alt_m;         // altitude above sea level, metres
    float   speed_mps;     // speed in metres per second
    float   heading_deg;   // true north heading, 0-360
    uint8_t fix_quality;   // 0=none, 1=GPS, 2=DGPS, 4=RTK fixed
    uint8_t num_sats;
    float   hdop;          // horizontal dilution of precision
    int64_t timestamp_ms;  // unix milliseconds (ms-of-day until first RMC supplies a date)
    bool    valid;
} NmeaFix_t;

typedef enum {
    NMEA_PARSE_OK,
    NMEA_PARSE_INVALID_CHECKSUM,
    NMEA_PARSE_UNKNOWN_SENTENCE,
    NMEA_PARSE_INCOMPLETE,
} NmeaParseResult_t;

// Parses one GGA or RMC sentence (any talker: GN/GP/GL/GA) into *out.
// Only the fields carried by that sentence type are written; initialize
// *out (or reuse the previous fix) so GGA+RMC pairs accumulate into one fix.
NmeaParseResult_t nmea_parse_sentence(const char *sentence, NmeaFix_t *out);

// XOR of all bytes between '$' and '*' must equal the two hex digits after '*'.
bool nmea_validate_checksum(const char *sentence);

// Reset parser-internal state (cached RMC date). Call between replay runs.
void nmea_parser_reset(void);

#ifdef __cplusplus
}
#endif

#endif // NAVGUARD_NMEA_PARSER_H
