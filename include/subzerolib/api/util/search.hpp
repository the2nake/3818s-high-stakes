#pragma once

#include <cmath>
#include <functional>
#include <vector>

// returns where x should go in the sorted vector
template <typename storage_t, typename compare_t>
int binary_search(
    std::vector<storage_t> &vec,
    compare_t x,
    std::function<compare_t(storage_t)> converter =
        [](storage_t s) -> compare_t { return s; }) {

  if (vec.size() == 0)
    return 0;

  int i_min = 0;
  int i_max = vec.size() - 1;
  if (converter(vec.front()) >= x)
    return 0;
  if (converter(vec[i_max]) < x)
    return i_max + 1;

  while (i_min <= i_max) {
    int i_mid = std::floor((i_min + i_max) / 2.0);
    compare_t min_val = converter(vec[i_min]);
    compare_t mid_val = converter(vec[i_mid]);
    compare_t max_val = converter(vec[i_max]);

    if (mid_val == x) {
      return i_mid;
    }

    if (min_val == x) {
      return i_min;
    } else if (i_min == i_mid - 1 && x <= mid_val) {
      return i_mid;
    } else if (i_mid == i_max - 1 && x > mid_val && x <= max_val) {
      return i_max;
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
