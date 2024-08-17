#include "subzerolib/api/geometry/trajectory-point.hpp"
#include "subzerolib/api/util/math.hpp"

template <>
trajectory_point_s lerp<trajectory_point_s, double>(trajectory_point_s a,
                                                    trajectory_point_s b,
                                                    double t) {
  auto delta_h = shorter_turn(a.h, b.h);
  return trajectory_point_s{lerp(a.t, b.t, t),
                            lerp(a.s, b.s, t),
                            lerp(a.x, b.x, t),
                            lerp(a.vx, b.vx, t),
                            lerp(a.y, b.y, t),
                            lerp(a.vy, b.vy, t),
                            a.h + delta_h / t,
                            lerp(a.vh, b.vh, t)};
}
