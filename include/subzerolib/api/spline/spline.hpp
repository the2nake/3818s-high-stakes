#pragma once

#include "subzerolib/api/geometry/pose.hpp"
#include "subzerolib/api/geometry/spline-point.hpp"

#include <eigen/Dense>
#include <vector>

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

  /// @brief gets the position
  /// @param u the parameter space value
  /// @returns a point_s with the position
  virtual point_s get_pos(double u) = 0;

  /// @brief gets the velocity
  /// @param u the parameter space value
  /// @returns a point_s with the velocity
  virtual point_s get_vel(double u) = 0;

  /// @brief gets the acceleration
  /// @param u the parameter space value
  /// @returns a point_s with the acceleration
  virtual point_s get_accel(double u) = 0;
};

std::vector<pose_s> interpolate_heading(std::vector<point_s> path,
                                        std::vector<pose_s> ctrl_points);
