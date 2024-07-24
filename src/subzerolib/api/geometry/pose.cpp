#include "subzerolib/api/geometry/pose.hpp"
#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/util/math.hpp"
#include <cmath>

double pose_s::dist(const point_s &other) const { return point().dist(other); }
double pose_s::dist(const pose_s &other) const {
  return std::hypot(x - other.x, y - other.y);
}

pose_s lerp(pose_s a, pose_s b, double t) {
  // clamp(factor, 0.0, 1.0)
  auto point = lerp(a.point(), b.point(), t);
  double heading = lerp(a.heading(), b.heading(), t);
  return pose_s{point, heading};
}

pose_s operator+(pose_s a, pose_s b) {
  return {a.x + b.x, a.y + b.y, a.h + b.h};
}

pose_s operator-(pose_s a, pose_s b) {
  return {a.x - b.x, a.y - b.y, shorter_turn<double>(b.h, a.h, 360.0)};
}
