#include "subzerolib/api/geometry/segment.hpp"

bool segment_s::may_contain(point_s point) const {
  bool x_condition = (start.x <= point.x && point.x <= end.x) ||
                     (end.x <= point.x && point.x <= start.x);
  bool y_condition = (start.y <= point.y && point.y <= end.y) ||
                     (end.y <= point.y && point.y <= start.y);

  return x_condition && y_condition;
}

double segment_s::slope() const {
  return (end.y - start.y) / (end.x - start.x);
}

point_s segment_s::intersection(const segment_s &other) const {
  point_s inf_intersection;
  auto d1 = end - start;
  auto d2 = other.end - other.start;

  // ix = end.x + d1.x * n = other.end.x + d2.x * m
  // iy = end.y + d1.y * n = other.end.y + d2.y * m
  // n = (other.end.x + d2.x * m - end.x) / (d1.x)
  // other.end.y + d2.y * m = end.y + d1.y * (other.end.x + d2.x * m - end.x)
  // / (d1.x) other.end.y + d2.y * m = end.y + d1.y * other.end.x / d1.x +
  // d1.y * d2.x * m / d1.x - d1.y * end.x / d1.x m * (d2.y - d1.y * d2.x /
  // d1.x) = end.y + d1.y * other.end.x / d1.x - d1.y * end.x / d1.x -
  // other.end.y

  double m =
      (end.y + d1.y * other.end.x / d1.x - d1.y * end.x / d1.x - other.end.y) /
      (d2.y - d1.y * d2.x / d1.x);
  inf_intersection = other.end + d2 * m;

  if (may_contain(inf_intersection) && other.may_contain(inf_intersection)) {
    return inf_intersection;
  } else {
    return {std::nan(""), std::nan("")};
  }
}