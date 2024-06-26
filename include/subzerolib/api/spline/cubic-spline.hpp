#pragma once

#include "subzerolib/api/geometry/point.hpp"

#include <vector>
#include "eigen/Dense"

class CubicSpline {
public:
  virtual ~CubicSpline() {}

  /// @brief generate points on a spline
  /// @param count number of points to sample
  /// @return a vector of evenly spaced points (in parameter space)
  virtual std::vector<point_s> sample(int counts) = 0;

  virtual point_s get_pos(double u) = 0;
  virtual point_s get_vel(double u) = 0;
  virtual point_s get_accel(double u) = 0;
};