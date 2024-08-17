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
  if (converter(vec[i_max]) == x)
    return i_max;
  if (converter(vec[i_max]) < x)
    return i_max + 1;

  int section = -1;

  // find the range
  while (i_max - i_min > 1) {
    int i_mid = (i_min + i_max) / 2;
    compare_t mid_val = converter(vec[i_mid]);
    // compare_t min_val = converter(vec[i_min]);
    // compare_t max_val = converter(vec[i_max]);

    if (x <= mid_val) {
      i_max = i_mid;
    } else {
      i_min = i_mid;
    }
  }

  return i_max;

  // this shouldn't ever be reached
  return -1;
}
