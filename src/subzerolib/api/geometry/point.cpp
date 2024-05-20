#include "subzerolib/api/geometry/point.hpp"

point_s operator+(point_s a, point_s b) { return {a.x + b.x, a.y + b.y}; }
point_s operator-(point_s a, point_s b) { return {a.x - b.x, a.y - b.y}; }

point_s operator*(double scale, point_s a) {
  return {a.x * scale, a.y * scale};
}
point_s operator*(point_s a, double scale) {
  return {a.x * scale, a.y * scale};
}
point_s operator/(point_s a, double invscale) {
  return {a.x / invscale, a.y / invscale};
}

point_s lerp(point_s a, point_s b, double t) { return (1 - t) * a + t * b; }

double point_s::dist(const point_s &b) const {
  return std::hypot(x - b.x, y - b.y);
}
