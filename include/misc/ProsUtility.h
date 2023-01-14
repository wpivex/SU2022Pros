#pragma once
#include "main.h"
#include "pros/motors.hpp"

inline void setEffort(pros::MotorGroup& motor, double effort) {
    int32_t mv = effort * 12000;
    motor.move_voltage(mv);
}

inline void setEffort(pros::Motor& motor, double effort) {
    int32_t mv = effort * 12000;
    motor.move_voltage(mv);
}