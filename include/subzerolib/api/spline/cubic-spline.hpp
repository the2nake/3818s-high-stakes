#pragma once

#include "subzerolib/api/geometry/point.hpp"

#include <vector>
#include "eigen/Dense"

class CubicSpline {
public:
  /// @brief generate points on a spline
  /// @param control_points are the control points of the spline
  /// @return a vector of evenly spaced points
  virtual std::vector<point_s> discretise(std::vector<point_s> control_points) = 0;

  virtual point_s get_pos(double t) = 0;
  virtual point_s get_vel(double t) = 0;
  virtual point_s get_accel(double t) = 0;
};