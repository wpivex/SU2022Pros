#pragma once

#include <vector>
#include <memory>
#include "Controller.h"
#include "Subsystems/Robot.h"

class PathFollower {

private:

    std::vector<std::vector<Waypoint>> path;
    std::unique_ptr<Controller> controller;
    Robot& robot;

public:

    PathFollower(Robot& r, Controller* controllerP):
        robot(r),
        controller(controllerP)
    {
        controller->initRobot(&robot);

    } // generate path from reading from file

    void runSegment(int index);

};