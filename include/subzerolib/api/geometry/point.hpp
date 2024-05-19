#pragma once

struct point_s {
  double x = 0;
  double y = 0;
};

point_s operator+(point_s a, point_s b);
point_s operator-(point_s a, point_s b);
point_s operator*(double scale, point_s a);
point_s operator*(point_s a, double scale);
point_s operator/(point_s a, double invscale);
point_s lerp(point_s a, point_s b, double t);