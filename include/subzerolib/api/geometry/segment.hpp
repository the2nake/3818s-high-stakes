#pragma once

#include "subzerolib/api/geometry/point.hpp"

struct segment_s {
  segment_s(point_s istart, point_s iend) : start(istart), end(iend) {}
  point_s start;
  point_s end;

  bool may_contain(point_s point) const;
  double slope() const;
  point_s intersection(const segment_s &other) const;
};