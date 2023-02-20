#pragma once
#include "Programs/TestFunction/AbstractTest.h"
#include "AutonomousFunctions/DriveFunctions.h"
#include "Algorithms/DoubleBoundedPID.h"

/*
0.8: P 0.123 D 0.027 Min 0.12

*/

class ForwardTest : public AbstractTest {

public:

    ForwardTest():
        AbstractTest(
            {0.1, 0.027, 0.12, 0.03, 0.075, 0.8},
            {"P", "D", "MIN", "ACCEL", "TOLERANCE", "SPEED"}
        )
    {}

    #define GFU_TURN SimplePID({1, 1.5, 0, 0.0, 1})
    double runFunction(Robot& robot) override {

        double p = paramValues[0];
        double d = paramValues[1];
        double min = paramValues[2];
        double accel = paramValues[3];
        double tolerance = paramValues[4];
        double maxSpeed = paramValues[5];

        double error = 0;
        std::vector<double> targets = {12, 36, -48};
        for (int counter = 0; counter < targets.size(); counter++) {

            DoubleBoundedPID forwardPID({p, 0, d, min, maxSpeed, accel}, tolerance, 3);

            error += fabs(goForwardU(robot, std::move(forwardPID), GFU_TURN, targets[counter], getRadians(0.0)));
        }
        return error;

    }
};