#pragma once

#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/util/math.hpp"

struct pose_s : public point_s {
  pose_s(point_s ipoint, double iheading = 0) : point_s(ipoint), h(iheading) {}
  pose_s(double ix = 0, double iy = 0, double ih = 0)
      : point_s(ix, iy), h(ih) {}

  double h;

  double heading() const { return h; }
  double radians() const { return in_rad(h); }
};

pose_s operator+(pose_s a, pose_s b);
pose_s operator-(pose_s a, pose_s b);

template <> pose_s lerp<pose_s, double>(pose_s a, pose_s b, double t);
