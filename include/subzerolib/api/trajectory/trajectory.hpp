#pragma once

#include "subzerolib/api/geometry/trajectory-point.hpp"

// TODO: write distance detection for pure pursuit
// TODO: new adaptive-pure-pursuit class?
// TODO: move structs to geometry

class Trajectory {
public:
  virtual double get_duration() = 0;
  virtual double get_length() = 0;
  virtual trajectory_point_s get_at_time(double t) = 0;
  virtual trajectory_point_s get_at_distance(double s) = 0;

protected:
  Trajectory() {}
};
