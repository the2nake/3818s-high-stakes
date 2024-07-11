#pragma once

#include "subzerolib/api/geometry/point.hpp"

#include "eigen/Dense"
#include <cmath>
#include <vector>

struct spline_point_s {
  spline_point_s(double ix = 0, double iy = 0, double is = 0, double ivx = 0,
                 double ivy = 0, double iax = 0, double iay = 0)
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
};

class Spline {
public:
  virtual ~Spline() {}

  /// @brief sample points on a spline
  /// @param count number of points to sample
  /// @return a vector of evenly spaced points (in parameter space)
  virtual std::vector<point_s> sample_coordinates(int count) = 0;

  /// @brief sample points with kinematic information on a spline
  /// @param count number of points to sample
  /// @return a vector of evenly spaced point data (in paramter space)
  virtual std::vector<spline_point_s> sample_kinematics(int count) = 0;

  virtual point_s get_pos(double u) = 0;
  virtual point_s get_vel(double u) = 0;
  virtual point_s get_accel(double u) = 0;
};