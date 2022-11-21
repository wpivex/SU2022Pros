#pragma once

#include <vector>
#include <memory>
#include "Controller.h"

class PathFollower {

private:

    std::vector<std::vector<Waypoint>> path;
    std::unique_ptr<Controller> controller;

public:

    PathFollower(Controller* controllerP):
        controller(controllerP)
    {} // generate path from reading from file

    void runSegment(int index);

};