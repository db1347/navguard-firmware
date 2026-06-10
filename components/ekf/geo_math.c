#include "geo_math.h"

#include <math.h>

double haversine_distance_m(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
                   sin(dlon / 2.0) * sin(dlon / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return NG_EARTH_RADIUS_M * c;
}

float wrap_heading_deg(float h) {
    h = fmodf(h, 360.0f);
    if (h < 0.0f) h += 360.0f;
    return h;
}

float heading_diff_deg(float a, float b) {
    float diff = fabsf(wrap_heading_deg(a) - wrap_heading_deg(b));
    if (diff > 180.0f) diff = 360.0f - diff;
    return diff;
}
