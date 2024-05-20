#pragma once

#include "eigen/Dense"

struct point_s {
  point_s(double ix = 0, double iy = 0) : x(ix), y(iy) {}
  double x;
  double y;

  void operator=(const point_s &b) {
    x = b.x;
    y = b.y;
  }

  void operator=(const Eigen::Matrix<double, 1, 2> &b) {
    x = b[0];
    y = b[1];
  }

  double dist(const point_s &b) const;
};

point_s operator+(point_s a, point_s b);
point_s operator-(point_s a, point_s b);
point_s operator*(double scale, point_s a);
point_s operator*(point_s a, double scale);
point_s operator/(point_s a, double invscale);
point_s lerp(point_s a, point_s b, double t);