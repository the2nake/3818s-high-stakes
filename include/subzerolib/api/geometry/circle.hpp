#pragma once

#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/geometry/pose.hpp"
#include "subzerolib/api/geometry/segment.hpp"

#include <vector>

struct circle_s {
  circle_s(point_s icentre, double iradius)
      : centre(icentre), radius(iradius) {}
  circle_s(pose_s icentre, double iradius)
      : centre(icentre.point()), radius(iradius) {}
  circle_s(double ix, double iy, double iradius)
      : centre(point_s{ix, iy}), radius(iradius) {}

  point_s centre;
  double radius = 1;

  bool contains(const point_s &point) const;
  std::vector<point_s> intersections(const segment_s &segment) const;
};