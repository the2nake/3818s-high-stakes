#pragma once

#include "subzerolib/api/geometry/point.hpp"

#include <cmath>

struct spline_point_s {
  spline_point_s(double ix = 0,
                 double iy = 0,
                 double is = 0,
                 double ivx = 0,
                 double ivy = 0,
                 double iax = 0,
                 double iay = 0)
      : x(ix), y(iy), s(is), vx(ivx), vy(ivy), ax(iax), ay(iay) {}
  double x;
  double y;

  double s; // distance

  double vx;
  double vy;
  double v() const { return std::hypot(vx, vy); }

  double ax;
  double ay;
  double a() const { return std::hypot(ax, ay); }

  point_s point() const { return {x, y}; }
};
