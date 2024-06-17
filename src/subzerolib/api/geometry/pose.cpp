#include "subzerolib/api/geometry/pose.hpp"
#include "subzerolib/api/geometry/point.hpp"
#include <cmath>

double pose_s::dist(const point_s &other) const {
  return point().dist(other);
}
double pose_s::dist(const pose_s &other) const {
  return std::hypot(x - other.x, y - other.y);
}
