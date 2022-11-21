#pragma once

#include "Drive.h"
#include "Flywheel.h"
#include "Localizer.h"
#include <memory>
#include "main.h"

struct Robot {

    std::unique_ptr<Drive> drive;
    std::unique_ptr<Localizer> localizer;
    std::unique_ptr<Flywheel> flywheel;

    pros::MotorGroup intake;
    pros::ADIDigitalOut indexer;

};