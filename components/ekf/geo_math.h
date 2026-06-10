#ifndef NAVGUARD_GEO_MATH_H
#define NAVGUARD_GEO_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#define NG_EARTH_RADIUS_M     6371000.0
#define NG_METERS_PER_DEG_LAT 111111.0

// Great-circle distance between two lat/lon points, metres.
double haversine_distance_m(double lat1, double lon1, double lat2, double lon2);

// Smallest absolute difference between two compass headings (0-360), degrees.
float heading_diff_deg(float a, float b);

// Normalize a heading into [0, 360).
float wrap_heading_deg(float h);

#ifdef __cplusplus
}
#endif

#endif // NAVGUARD_GEO_MATH_H
