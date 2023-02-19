#pragma once
#include "Programs/TestFunction/AbstractTest.h"
#include "AutonomousFunctions/DriveFunctions.h"
#include "Algorithms/DoubleBoundedPID.h"

class TurnTest : public AbstractTest {

public:

    TurnTest():
        AbstractTest(
            {1.25, 0.005, 0.13, 0.17},
            {"P", "I", "D", "MIN"}
        )
    {}

    void runFunction(Robot& robot) override {

        double p = paramValues[0];
        double i = paramValues[1];
        double d = paramValues[2];
        double min = paramValues[3];

        std::vector<double> targets = {90, 180, 270, 0};
        for (int counter = 0; counter < targets.size(); i++) {
            DoubleBoundedPID gtu_turn_precise({p, i, d, min, 1}, getRadians(0.5), 3);
            goTurnU(robot, std::move(gtu_turn_precise), getRadians(targets[counter]));
        }

    }
};