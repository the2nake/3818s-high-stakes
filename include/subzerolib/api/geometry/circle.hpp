#pragma once

#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/geometry/segment.hpp"
#include <vector>

struct circle_s {
  point_s centre;
  double radius = 1;

  bool contains(const point_s &point) const;
  std::vector<point_s> intersections(const segment_s &segment) const;
};