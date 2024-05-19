#pragma once

#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/spline/cubic-spline.hpp"
#include <vector>

class CatmullRomSpline : public CubicSpline {
public:
  CatmullRomSpline();

  std::vector<point_s> discretise(std::vector<point_s> control_points) override;

  point_s get_pos(double t) override;
  point_s get_vel(double t) override;
  point_s get_accel(double t) override;

  /// @brief adds points to the ends of the catmull-rom spline
  /// @param v0 a point representing the velocity at the initial waypoint
  /// @param vf a point representing the velocity at the final waypoint
  /// @returns a vector of control points for use with generation
  static std::vector<point_s> pad_velocity(std::vector<point_s> waypoints,
                                           point_s v0, point_s vf);

private:
  // x = [1 t t^2  t^3][characteristic_matrix][c1 c2 c3 c4]
  // v = [0 1  2t 3t^2][characteristic_matrix][c1 c2 c3 c4]
  // a = [0 0   2   6t][characteristic_matrix][c1 c2 c3 c4]
  const Eigen::Matrix<double, 4, 4> characteristic_matrix{
      {0, 1, 0, 0},
      {-0.5, 0, 0.5, 0},
      {1, -2.5, 2, -0.5},
      {-0.5, 1.5, -1.5, 0.5}};
};