#pragma once

#include <vector>
#include <limits>
#include "math.h"
#include "config.h"

#define POS_INF (1.0 / 0.0)
#define NEG_INF ((-1.0) / 0.0)

#define ERROR_DOUBLE (2048)
#define METERS_TO_INCHES (39.3701)

// return -1 to 1, cubed, with deadzone
inline double normalizedAxis(pros::controller_analog_e_t joystick) {
    int32_t raw = controller.get_analog(joystick); // between -127 and 127

    double normalized = raw / 127.0;
    if (fabs(normalized) < 0.01) normalized = 0;

    return pow(normalized, 3);
}


inline double average(const std::vector<double>& nums) {
    double sum = 0;
    for (double num : nums) sum += num;
    return sum / nums.size();
}

inline double clamp(double value, double min, double max) {
    return fmin(max, fmax(min, value));
}

inline int sign(double x) {
    if (x >= 0) return 1;
    return -1;
}

// Bound angle to between -pi and pi, preferring the smaller magnitude
inline double boundAngleRadians(double angle) {
    angle = fmod(angle, M_PI*2);
    if (angle < -M_PI) angle += 2*M_PI;
    if (angle > M_PI) angle -= 2*M_PI;
    return angle;
}

// Find the closest angle between two universal angles
inline double deltaInHeading(double targetHeading, double currentHeading) {
  return boundAngleRadians(targetHeading - currentHeading);
}

constexpr double getRadians(double degrees) {
    return degrees / 180.0 * M_PI;
}

constexpr double getDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

// Get distance from (x1, y1) to (x2, y2)
inline double getDistance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx*dx + dy*dy);
}

// Return distance from (x1, y1) to the closest point to (x,y) on line from (x1, y1) to (x2, y2)
inline double distanceToPointProjection(double x, double y, double x1, double y1, double x2, double y2) {
    double ax = x - x1;
    double ay = y - y1;
    double bx = x2 - x1;
    double by = y2 - y1;

    double scalar = (ax * bx + ay * by) / (bx * bx + by * by);
    return getDistance(x1, y1, x1 + scalar * bx, y1 + scalar * by);

}

inline double headingToPoint(double startX, double startY, double goalX, double goalY) {
    return atan2(goalY - startY, goalX - startX) - M_PI/2;
}

// Distance between point (x0, y0) and line (x1, y1,),(x2,y2)
inline double distancePointToLine(double x0, double y0, double x1, double y1, double x2, double y2) {
    return ((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1)) / getDistance(x1, y1, x2, y2);
}