#pragma once
#include <vector>
#include "Subsystems/Robot.h"

class AbstractTest {

public:

    std::vector<double> paramValues;
    std::vector<std::string> paramNames;

    AbstractTest(std::initializer_list<double> paramValues, std::initializer_list<std::string> paramNames):
        paramValues(paramValues), paramNames(paramNames) {}

    virtual void runFunction(Robot& robot) = 0;
    
    double run(Robot& robot) {
        int32_t startTime = pros::millis();

        runFunction(robot);

        double time = ((double) (pros::millis() - startTime)) / 1000.0;
        return time;
    }

};