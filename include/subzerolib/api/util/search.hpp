#pragma once

#include <cmath>
#include <functional>
#include <vector>

#include "stdio.h"

// TODO: unit test

// returns where x should go in the sorted vector
template <typename StorageType, typename CompareType>
int binary_search(
    std::vector<StorageType> &vec,
    CompareType x,
    std::function<CompareType(StorageType)> converter =
        [](StorageType s) -> CompareType { return s; }) {

  if (vec.size() == 0)
    return 0;

  int i_min = 0;
  int i_max = vec.size() - 1;
  if (converter(vec.front()) >= x)
    return 0;
  if (converter(vec[i_max]) <= x)
    return i_max;

  while (i_min <= i_max) {
    int i_mid = std::floor((i_min + i_max) / 2.0);
    CompareType min_val = converter(vec[i_min]);
    CompareType mid_val = converter(vec[i_mid]);
    CompareType max_val = converter(vec[i_max]);

    if (mid_val == x) {
      return i_mid;
    }

    if (min_val == x) {
      return i_min;
    } else if (i_min == i_mid - 1 && x <= mid_val) {
      return mid_val;
    } else if (i_mid == i_max - 1 && x > mid_val && x <= max_val) {
      return max_val;
    }

    if (mid_val > x) {
      i_max = i_mid - 1;
    } else {
      i_min = i_mid + 1;
    }
  }

  // this shouldn't ever be reached
  return -1;
}
