#ifndef NAVGUARD_NMEA_BUILDER_H
#define NAVGUARD_NMEA_BUILDER_H

#include <stddef.h>
#include "nmea_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

// Build a $GNGGA sentence (with checksum + CRLF) from *fix into out.
// Returns number of bytes written (excluding NUL), or -1 on buffer overflow.
int nmea_build_gngga(const NmeaFix_t *fix, char *out, size_t out_len);

// Build a $GNRMC sentence (with checksum + CRLF) from *fix into out.
int nmea_build_gnrmc(const NmeaFix_t *fix, char *out, size_t out_len);

// Append the "*HH\r\n" checksum suffix to a sentence body that already
// starts with '$' and has no '*'. Returns bytes written or -1.
int nmea_append_checksum(char *sentence, size_t out_len);

#ifdef __cplusplus
}
#endif

#endif // NAVGUARD_NMEA_BUILDER_H
