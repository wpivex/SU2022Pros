#pragma once

class Flywheel {

public:
    virtual void setVelocity(double velocity) = 0;
    virtual double getTargetVelocity() = 0;
    virtual double getCurrentVelocity() = 0;
    virtual void maintainVelocityTask() = 0;
    virtual bool atTargetVelocity() = 0;
    virtual void setRawVoltage(double volts) = 0;
};