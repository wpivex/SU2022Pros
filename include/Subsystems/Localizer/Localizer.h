#pragma once

class Localizer {

public:
    virtual double getX() {return 0;} // inches
    virtual double getY() {return 0;} // inches
    virtual double getHeading() {return 0;} // radians
    
    virtual void updatePositionTask() {} // blocking task used to update (x, y, heading)
    virtual void init() {};
    virtual void setPosition(double x, double y) {}
    virtual void setHeading(double headingRadians) {}
};