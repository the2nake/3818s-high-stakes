#include "subzerolib/api/geometry/circle.hpp"
#include "subzerolib/api/geometry/point.hpp"

bool circle_s::contains(const point_s &point) const {
  return point.dist(centre) <= radius;
}

std::vector<point_s> circle_s::intersections(const segment_s &segment) const {
  // (x - centre.x) ^ 2 + (y - centre.y) ^ 2 = radius ^ 2
  // m = segment.slope()
  // y - segment.start.y = m * x - m * segment.start.x
  // y = m * x - m * segment.start.x + segment.start.y
  // (x - centre.x) ^ 2 + (m * x - m * segment.start.x + segment.start.y -
  //   centre.y) ^ 2 = radius ^ 2

  // c = (segment.start.y - m * segment.start.x - centre.y)
  // (x - centre.x) ^ 2 + (m * x + c) ^ 2 = radius ^ 2
  // x^2 - 2*centre.x*x + centre.x^2 + m^2x^2 + 2*m*x*c + c^2 - radius^2 = 0
  // (m^2+1)x^2 + (2*m*c - 2*centre.x)*x + c^2 - radius^2 + centre.x^2 = 0
  double m = segment.slope();
  double d = segment.start.y - m * segment.start.x - centre.y;
  double a = m * m + 1;
  double b = 2 * m * d - 2 * centre.x;
  double c = d * d - radius * radius + centre.x * centre.x;

  double x1 = (-b + std::sqrt(b * b - 4 * a * c)) / (2 * a);
  double x2 = (-b - std::sqrt(b * b - 4 * a * c)) / (2 * a);
  double y1 = m * x1 - m * segment.start.x + segment.start.y;
  double y2 = m * x2 - m * segment.start.x + segment.start.y;

  std::vector<point_s> candidates;
  candidates.emplace_back(x1, y1);
  candidates.emplace_back(x2, y2);

  std::vector<point_s> output;

  for (auto &candidate : candidates) {
    if (segment.may_contain(candidate)) {
      output.push_back(candidate);
    }
  }

  return output;
}