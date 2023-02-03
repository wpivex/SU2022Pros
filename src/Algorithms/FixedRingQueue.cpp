#include "Algorithms/FixedRingQueue.h"
#include "math.h"

//template <typename T>

RingQueue::RingQueue(int sizeP) {
  capacity = sizeP;
  arr = std::vector<double>(sizeP);
}

// If at capacity, the first element is popped
bool RingQueue::push(double value) {

  sum += value; // append to sum

  if (size < capacity) {
    arr[size] = value;
    size++;
    return false;
  } else {
    sum -= arr[firstElement]; // since popping last element, subtract from sum
    arr[firstElement] = value;
    firstElement = (firstElement + 1) % capacity;
    return true;
  }
}

double RingQueue::getAverage() {

  if (size == 0) return 0; 

  
  
  return sum / size;
}

double RingQueue::get(int index) {
  return arr[(firstElement + index) % capacity];
}

int RingQueue::getSize() {
  return size;
}

double RingQueue::standardDeviation() {

  if (size == 0) return 0;

  double sum = 0; // recalc sum to avoid drift
  int i = firstElement;
  int count = 0;
  while (count < size) {
    sum += arr[i];
    i = (i + 1) % capacity;
    count++;
  }
  double mean = sum / size;

  double sumsqr = 0; // sum squared
  i = firstElement;
  count = 0;
  while (count < size) {
    double deviation = arr[i] - mean;
    sumsqr += deviation*deviation;
    i = (i + 1) % capacity;
    count++;
  }

  return sqrt(sumsqr / size);
  
}

double RingQueue::isAllEqual() {

  double num = arr[firstElement];

  int i = (firstElement + 1) % capacity;
  int count = 1;
  while (count < size) {
    
    if (num != arr[i]) return false;

    i = (i + 1) % capacity;
    count++;
  }

  return size == capacity;
}