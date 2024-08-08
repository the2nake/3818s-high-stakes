#pragma once

#include "subzerolib/api/spline/spline.hpp"

// TODO: write distance detection for pure pursuit
// TODO: new adaptive-pure-pursuit class?
// TODO: move structs to geometry

struct trajectory_point_s {
  trajectory_point_s(double i_t = 0,
                     double i_s = 0,
                     double i_x = 0,
                     double i_vx = 0,
                     double i_y = 0,
                     double i_vy = 0,
                     double i_h = 0,
                     double i_vh = 0)
      : t(i_t), s(i_s), x(i_x), vx(i_vx), y(i_y), vy(i_vy), h(i_h), vh(i_vh) {}
  trajectory_point_s(spline_point_s &point)
      : s(point.s), x(point.x), vx(point.vx), y(point.y), vy(point.vy) {}

  double t = 0;
  double s = 0;
  double x = 0;
  double vx = 0;
  double y = 0;
  double vy = 0;
  double h = 0;
  double vh = 0;
};

class Trajectory {
public:
  virtual double get_duration() = 0;
  virtual double get_length() = 0;
  virtual trajectory_point_s get_at_time(double t) = 0;
  virtual trajectory_point_s get_at_distance(double s) = 0;

protected:
  Trajectory() {}
};
