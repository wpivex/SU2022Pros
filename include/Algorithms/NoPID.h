#pragma once

#include "SingleBoundedPID.h"

/*
Not actually a PID controller.
Instead, it's a binary step controller that outputs max power until target reached.
*/
class NoPID : public SingleBoundedPID {


public:

    NoPID(double speed):
        SingleBoundedPID({speed, 0, 0})
    {
        stopMotors = false;
    }

    double tick(double error) override;

};