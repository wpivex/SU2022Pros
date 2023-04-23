#pragma once

typedef struct PIDParameters {
  double P, I, D;
  double MIN, MAX;
  double MAX_ACCEL;

  PIDParameters(double kp, double ki, double kd, double minMagnitude, double maxMagnitude, double maxAcceleration):
    P(kp), I(ki), D(kd), MIN(minMagnitude), MAX(maxMagnitude), MAX_ACCEL(maxAcceleration) {}

  PIDParameters(double kp, double ki, double kd, double minMagnitude, double maxMagnitude):
    PIDParameters(kp, ki, kd, minMagnitude, maxMagnitude, 1000000) {}


  PIDParameters(double kp, double ki, double kd):
    PIDParameters(kp, ki, kd, 0, 1000000) {}

} PIDParameters;

/*
A PID controller with configurable parameters. Does not have termination logic
*/
class SimplePID {

public:

  SimplePID(PIDParameters params, bool limitAcceleration = true): K(params), limitAccel(limitAcceleration) {}
  virtual double tick(double error);
  void setNewParam(double kp, double ki, double kd);
  double getCurrentError();
protected:
  virtual void handleEndCondition(double error) {}

  double prevError = 0;
  double prevIntegral = 0;
  double prevOutput = 0;
  
  PIDParameters K;
  bool limitAccel;

};
