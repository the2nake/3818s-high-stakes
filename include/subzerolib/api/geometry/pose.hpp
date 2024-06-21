#pragma once

#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/util/math.hpp"

struct pose_s {
  pose_s(point_s ipoint, double iheading) : x(ipoint.x), y(ipoint.y), h(iheading) {}
  pose_s(double ix = 0, double iy = 0, double ih = 0) : x(ix), y(iy), h(ih) {}

  double x;
  double y;
  double h;

  point_s point() const { return point_s{x, y}; }

  double heading() const { return h; }
  double radians() const { return in_rad(h); }

  double dist(const point_s &other) const;
  double dist(const pose_s &other) const;
};

pose_s lerp(pose_s a, pose_s b, double t);
