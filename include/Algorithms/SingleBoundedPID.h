#pragma once

#include "EndablePID.h"

/*
A PID controller which is bounded with a one-sided threshold. This means that when the threshold is crossed,
the controller terminates
*/
class SingleBoundedPID : public EndablePID {

public:

    SingleBoundedPID(PIDParameters params, bool limitAcceleration = true):
        EndablePID(params, limitAcceleration)
    {}
    bool isCompleted() override;

protected:
    void handleEndCondition(double error) override;
    bool goingUp;
    bool isFirst = true;
    bool done = false;
};