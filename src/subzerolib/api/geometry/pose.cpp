#include "subzerolib/api/geometry/pose.hpp"
#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/util/math.hpp"
#include <cmath>

template <> pose_s lerp<pose_s, double>(pose_s a, pose_s b, double t) {
  auto point = lerp<point_s>(a, b, t);
  double heading = a.h + shorter_turn(a.h, b.h) * t;
  return pose_s{point, heading};
}

pose_s operator+(pose_s a, pose_s b) {
  return {a.x + b.x, a.y + b.y, a.h + b.h};
}

pose_s operator-(pose_s a, pose_s b) {
  return {a.x - b.x, a.y - b.y, shorter_turn<double>(b.h, a.h, 360.0)};
}
