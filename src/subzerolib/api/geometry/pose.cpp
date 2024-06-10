#include "subzerolib/api/geometry/pose.hpp"

#include <cmath>

double pose_s::dist(const pose_s &other) const {
  return std::hypot(x - other.x, y - other.y);
}
