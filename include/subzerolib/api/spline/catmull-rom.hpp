#pragma once

#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/geometry/pose.hpp"
#include "subzerolib/api/spline/cubic-spline.hpp"

#include <vector>

class CatmullRomSpline : public Spline {
public:
  /// @brief create a catmull-rom spline
  /// @param control_points a vector of control points
  /// @returns a catmull-rom spline object
  CatmullRomSpline(std::vector<point_s> control_points);

  /// @brief create a catmull-rom spline
  /// @param control_points a vector of poses representing control points
  /// @returns a catmull-rom spline object
  CatmullRomSpline(std::vector<pose_s> control_points);
  ~CatmullRomSpline() {}

  /// @brief sample points on a spline
  /// @param count number of points to sample
  /// @return a vector of evenly spaced points (in parameter space)
  std::vector<point_s> sample_coordinates(int count) override;

  /// @brief sample points with kinematic information on a spline
  /// @param count number of points to sample
  /// @return a vector of evenly spaced point data (in paramter space)
  std::vector<spline_point_s> sample_kinematics(int count) override;

  /// @brief gets the position
  /// @param u the parameter space value
  /// @returns a point_s with the position
  point_s get_pos(double u) override;

  /// @brief gets the velocity
  /// @param u the parameter space value
  /// @returns a point_s with the velocity
  point_s get_vel(double u) override;

  /// @brief gets the acceleration
  /// @param u the parameter space value
  /// @returns a point_s with the acceleration
  point_s get_accel(double u) override;

  /// @brief adds points to the ends of the catmull-rom spline
  /// @param v0 a point representing the velocity at the initial waypoint
  /// @param vf a point representing the velocity at the final waypoint
  void pad_velocity(point_s v0, point_s vf);

  /// @brief gets the control points for the spline
  /// @returns a vector of control points
  std::vector<point_s> get_control_points() { return control_points; }

private:
  void calculate_bernstein_coeffs();
  point_s get_val(Eigen::Matrix<double, 1, 4> t_row, double u);

  // x = [1 t t^2  t^3][characteristic_matrix][c1 c2 c3 c4]
  // v = [0 1  2t 3t^2][characteristic_matrix][c1 c2 c3 c4]
  // a = [0 0   2   6t][characteristic_matrix][c1 c2 c3 c4]
  const Eigen::Matrix<double, 4, 4> characteristic_matrix{
      {0, 1, 0, 0},
      {-0.5, 0, 0.5, 0},
      {1, -2.5, 2, -0.5},
      {-0.5, 1.5, -1.5, 0.5}};

  std::vector<Eigen::Matrix<double, 4, 2>> bernstein_coeffs;

  std::vector<point_s> control_points;
};