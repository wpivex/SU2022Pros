#pragma once

#include "EndablePID.h"

/*
A PID controller which is bounded with an interval. If the error falls between an interval for some number
of times in a row, then the controller terminates
*/
class DoubleBoundedPID : public EndablePID {

public:

    DoubleBoundedPID(PIDParameters params, double errorTolerance, int timesWithinTolerance, bool limitAcceleration = true):
        EndablePID(params, limitAcceleration),
        tolerance(errorTolerance),
        timesNeeded(timesWithinTolerance)
    {}
    bool isCompleted() override;

private:
    void handleEndCondition(double error) override;
    double tolerance;
    int timesNeeded;
    int times = 0;
};
