#include "subzerolib/api/spline/catmull-rom.hpp"

std::vector<point_s>
CatmullRomSpline::pad_velocity(std::vector<point_s> waypoints, point_s v0,
                               point_s vf) {
  std::vector<point_s> output = waypoints;
  point_s start_padding = *(waypoints.begin() + 1) - v0;
  point_s end_padding = *(waypoints.end() - 1) + vf;
  output.insert(output.begin(), start_padding);
  output.insert(output.end(), end_padding);
  return output;
}