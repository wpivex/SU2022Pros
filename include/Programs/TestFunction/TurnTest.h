#pragma once
#include "Programs/TestFunction/AbstractTest.h"
#include "AutonomousFunctions/DriveFunctions.h"
#include "Algorithms/DoubleBoundedPID.h"

class TurnTest : public AbstractTest {

public:

    TurnTest():
        AbstractTest(
            {1.25, 0.005, 0.13, 0.17, 0.5},
            {"P", "I", "D", "MIN", "TOLERANCE"}
        )
    {}

    double runFunction(Robot& robot) override {

        double p = paramValues[0];
        double i = paramValues[1];
        double d = paramValues[2];
        double min = paramValues[3];
        double tolerance = paramValues[4];

        double error = 0;
        std::vector<double> targets = {30, 90, 180, 0};
        for (int counter = 0; counter < targets.size(); counter++) {
            DoubleBoundedPID gtu_turn_precise({p, i, d, min, 1}, getRadians(tolerance), 3);

            double rad = getRadians(targets[counter]);
            goTurnU(robot, std::move(gtu_turn_precise), rad);
            error += fabs(deltaInHeading(rad, robot.localizer->getHeading()));
        }
        return getDegrees(error);

    }
};