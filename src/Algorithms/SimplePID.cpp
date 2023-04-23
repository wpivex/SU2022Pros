#include "Algorithms/SimplePID.h"
#include "math.h"

double SimplePID::tick(double error) {

  handleEndCondition(error);

  double integral = prevIntegral + error * 0.02;
  double derivative = (error - prevError) / 0.02;

  double output = K.P * error + K.I * integral + K.D * derivative;
  prevError = error;
  prevIntegral = integral;

  // Set mininum output value
  if (output > 0) {
    output = fmax(K.MIN, output);
  } else {
    output = fmin(-K.MIN, output);
  }
  output = fmax(-K.MAX, fmin(K.MAX, output));

  // Bound output by maximum acceleration
  if (limitAccel) output = fmax(prevOutput - K.MAX_ACCEL, fmin(prevOutput + K.MAX_ACCEL, output));

  prevOutput = output;

  return output;
}

void SimplePID::setNewParam(double kp, double ki, double kd){
  K = PIDParameters(kp,ki,kd);
}

double SimplePID::getCurrentError() {
  return prevError;
}