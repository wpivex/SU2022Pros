#pragma once

#include "Subsystems/Drive/Drive.h"
#include "Subsystems/Flywheel/Flywheel.h"
#include "Subsystems/Localizer/Localizer.h"
#include <memory>
#include "main.h"

class Robot {

public:

    std::unique_ptr<Drive> drive;
    std::unique_ptr<Localizer> localizer;
    std::unique_ptr<Flywheel> flywheel;

    std::unique_ptr<pros::MotorGroup> intake;
    std::unique_ptr<pros::ADIDigitalOut> indexer;

    std::unique_ptr<pros::MotorGroup> cata;
    std::unique_ptr<pros::ADIDigitalIn> limitSwitch;

    std::unique_ptr<pros::Motor> roller;
    std::unique_ptr<pros::ADIDigitalOut> shooterFlap;

    std::unique_ptr<pros::ADIDigitalOut> endgame;

};