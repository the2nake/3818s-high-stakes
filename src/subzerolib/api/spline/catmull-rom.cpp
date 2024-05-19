#include "subzerolib/api/spline/catmull-rom.hpp"
#include <iostream>

CatmullRomSpline::CatmullRomSpline(std::vector<point_s> icontrol_points)
    : control_points(icontrol_points) {
  // calculate cache for control points (every set of 4)
  calculate_bernstein_coeffs();
}

void CatmullRomSpline::calculate_bernstein_coeffs() {
  bernstein_coeffs.clear();
  if (control_points.size() >= 4) {
    for (int end_i = 3; end_i < control_points.size(); ++end_i) {
      Eigen::Matrix<double, 4, 2> ctrl_matrix;
      for (int inc = 3; inc >= 0; --inc) {
        ctrl_matrix.row(3 - inc)[0] = control_points[end_i - inc].x;
        ctrl_matrix.row(3 - inc)[1] = control_points[end_i - inc].y;
      }
      bernstein_coeffs.push_back(characteristic_matrix * ctrl_matrix);
    };
  }

  std::cout << bernstein_coeffs.size();
}

std::vector<point_s> CatmullRomSpline::sample(int count) {
  std::vector<point_s> output;
  if (count < 1) {
    return output;
  }
  int segments = control_points.size() - 3;
  double step = (double)(segments) / (count - 1);
  double u = 0.0;
  while (u < segments + 0.0001) {
    output.emplace_back(get_pos(u));
    u += step;
  }

  return output;
}

point_s CatmullRomSpline::get_val(Eigen::Matrix<double, 1, 4> t_row, double u) {
  point_s out{0, 0};
  if (u < 0.0 || u > control_points.size() - 3 + 0.001 ||
      bernstein_coeffs.size() == 0) {
    return out;
  }

  int whole = (int)std::floor(u) - (whole == bernstein_coeffs.size());

  Eigen::Matrix<double, 1, 2> product = t_row * bernstein_coeffs[whole];
  out = product;

  return out;
}

point_s CatmullRomSpline::get_pos(double u) {
  double t = u - std::floor(u);
  return get_val(Eigen::Matrix<double, 1, 4>{1, t, t * t, t * t * t}, u);
}

point_s CatmullRomSpline::get_vel(double u) {
  double t = u - std::floor(u);
  return get_val(Eigen::Matrix<double, 1, 4>{0, 1, 2 * t, 3 * t * t}, u);
}

point_s CatmullRomSpline::get_accel(double u) {
  double t = u - std::floor(u);
  return get_val(Eigen::Matrix<double, 1, 4>{0, 0, 2, 6 * t}, u);
}

void CatmullRomSpline::pad_velocity(point_s v0, point_s vf) {
  if (control_points.size() >= 2) {
    point_s start_padding = *(control_points.begin() + 1) - 2 * v0;
    point_s end_padding = *(control_points.end() - 2) + 2 * vf;
    control_points.insert(control_points.begin(), start_padding);
    control_points.push_back(end_padding);
  }
  calculate_bernstein_coeffs();
}