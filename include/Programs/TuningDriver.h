#pragma once
#include "Programs/Driver.h"
#include "pros/misc.h"
#include "Programs/TestFunction/AbstractTest.h"
#include <memory>
#include <vector>

class TuningDriver : public Driver {

public:

    TuningDriver(Robot& robot, std::unique_ptr<AbstractTest> test):
        Driver(robot),
        test(std::move(test))
    {}

    void runDriver() override;
        
private:

    std::unique_ptr<AbstractTest> test;

    void drawAdjustableParameters(int line, int numParams, int selectedParam, std::vector<double>& paramValues, std::vector<std::string>& paramNames, TestData& data);
    void handleControllerInput(int numParams, int& selectedParam, std::vector<double>& paramValues);


};