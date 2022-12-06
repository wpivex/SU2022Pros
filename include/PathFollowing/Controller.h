#pragma once
#include <vector>
#include "Subsystems/Robot.h"
#include "Waypoint.h"

class Controller {

protected:
    Robot* robot;
public:
    void initRobot(Robot* robotP) {robot = robotP; }
    virtual void runSegment(std::vector<Waypoint>& path) = 0;
};