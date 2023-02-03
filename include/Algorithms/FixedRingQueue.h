#pragma once

#include <vector>


class RingQueue {

  public:
  RingQueue(int sizeP);
  
  bool push(double value); // Push to queue. If at capacity, pop.
  double getAverage();
  double get(int index);
  int getSize();
  double standardDeviation();

  double isAllEqual();

  private:
  std::vector<double> arr;
  int capacity;
  int size = 0;
  int firstElement = 0;
  double sum = 0;
};