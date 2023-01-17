#pragma once

class Localizer {

public:
    virtual double getX() = 0; // inches
    virtual double getY() = 0; // inches
    virtual double getHeading() = 0; // radians
    virtual double getRotation() = 0;
    
    virtual void updatePositionTask() = 0; // blocking task used to update (x, y, heading)
    virtual void init() = 0;
    virtual void setPosition(double x, double y) = 0;
    virtual void setHeading(double headingRadians) = 0;
    virtual void setRotation(double rotationRadians) = 0;
};