#pragma once

typedef struct PIDParameters {
  float P, I, D;
  float MIN, MAX;
  float MAX_ACCEL;

  PIDParameters(float kp, float ki, float kd, float minMagnitude, float maxMagnitude, float maxAcceleration):
    P(kp), I(ki), D(kd), MIN(minMagnitude), MAX(maxMagnitude), MAX_ACCEL(maxAcceleration) {}

  PIDParameters(float kp, float ki, float kd, float minMagnitude, float maxMagnitude):
    PIDParameters(kp, ki, kd, minMagnitude, maxMagnitude, 1000000) {}


  PIDParameters(float kp, float ki, float kd):
    PIDParameters(kp, ki, kd, 0, 1000000) {}

} PIDParameters;

/*
A PID controller with configurable parameters. Does not have termination logic
*/
class SimplePID {

public:

  SimplePID(PIDParameters params): K(params) {}
  virtual float tick(float error);
  void setNewParam(float kp, float ki, float kd);
protected:
  virtual void handleEndCondition(float error) {}

  float prevError = 0;
  float prevIntegral = 0;
  float prevOutput = 0;
  
  PIDParameters K;

};
