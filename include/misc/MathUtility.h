#pragma once

#include <vector>
#include "math.h"
#include "config.h"

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