#pragma once
#include "math.h"

typedef struct Waypoint {
    double x, y;
} Waypoint;

double distance(const Waypoint& a, const Waypoint& b) {
    double x = b.x - a.x;
    double y = b.y - a.y
    return math.sqrt(x*x + y*y);
}