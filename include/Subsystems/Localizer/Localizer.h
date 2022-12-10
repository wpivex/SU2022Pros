#pragma once

class Localizer {

public:
    virtual double getX() = 0; // inches
    virtual double getY() = 0; // inches
    virtual double getHeading() = 0; // radians
    
    virtual void updatePositionTask() = 0; // blocking task used to update (x, y, heading)
    virtual void init() {}
    virtual void setPosition(double x, double y, double headingRadians);
};