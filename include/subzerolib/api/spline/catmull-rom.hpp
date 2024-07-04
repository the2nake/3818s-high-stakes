#pragma once

#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/geometry/pose.hpp"
#include "subzerolib/api/spline/cubic-spline.hpp"

#include <vector>

class CatmullRomSpline : public Spline {
public:
  CatmullRomSpline(std::vector<point_s> control_points);
  CatmullRomSpline(std::vector<pose_s> control_points);
  ~CatmullRomSpline() {}

  std::vector<point_s> sample(int count) override;

  point_s get_pos(double u) override;
  point_s get_vel(double u) override;
  point_s get_accel(double u) override;

  /// @brief adds points to the ends of the catmull-rom spline
  /// @param v0 a point representing the velocity at the initial waypoint
  /// @param vf a point representing the velocity at the final waypoint
  /// @returns a vector of control points for use with generation
  void pad_velocity(point_s v0, point_s vf);

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