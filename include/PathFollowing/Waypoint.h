#pragma once
#include "math.h"

typedef struct Waypoint {
    double x, y;
} Waypoint;

inline double distance(const Waypoint& a, const Waypoint& b) {
    double x = b.x - a.x;
    double y = b.y - a.y;
    return sqrt(x*x + y*y);
}

inline double thetaBetweenWaypoints(const Waypoint& a, const Waypoint& b) {
    return atan2(b.y - a.y, b.x - a.x);
}