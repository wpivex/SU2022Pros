#pragma once

#include <vector>

inline double average(const std::vector<double>& nums) {
    double sum = 0;
    for (double num : nums) sum += num;
    return sum / nums.size();
}