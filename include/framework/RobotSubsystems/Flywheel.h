#pragma once

class Flywheel {

public:
    virtual void setVelocity(double velocity) = 0;
    virtual double getVelocity() = 0;
    virtual void maintainVelocityTask() = 0;
};