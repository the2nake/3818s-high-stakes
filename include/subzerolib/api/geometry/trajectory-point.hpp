#pragma once

#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/geometry/spline-point.hpp"

struct trajectory_point_s : public point_s {
  trajectory_point_s(double i_t = 0,
                     double i_s = 0,
                     double i_x = 0,
                     double i_vx = 0,
                     double i_y = 0,
                     double i_vy = 0,
                     double i_h = 0,
                     double i_vh = 0)
      : point_s(i_x, i_y), t(i_t), s(i_s), vx(i_vx), vy(i_vy), h(i_h),
        vh(i_vh) {}
  trajectory_point_s(spline_point_s &point)
      : s(point.s), point_s(point.x, point.y), vx(point.vx), vy(point.vy) {}

  double t = 0;
  double s = 0;
  double vx = 0;
  double vy = 0;
  double h = 0;
  double vh = 0;
};

template <>
trajectory_point_s lerp<trajectory_point_s, double>(trajectory_point_s a,
                                                    trajectory_point_s b,
                                                    double t);
