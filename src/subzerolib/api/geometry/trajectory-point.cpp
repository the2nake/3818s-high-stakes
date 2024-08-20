#include "subzerolib/api/geometry/trajectory-point.hpp"

template <>
trajectory_point_s lerp<trajectory_point_s, double>(trajectory_point_s a,
                                                    trajectory_point_s b,
                                                    double t) {
  return trajectory_point_s{lerp(a.t, b.t, t),
                            lerp(a.s, b.s, t),
                            lerp(a.x, b.x, t),
                            lerp(a.vx, b.vx, t),
                            lerp(a.y, b.y, t),
                            lerp(a.vy, b.vy, t),
                            lerp(a.h, b.h, t),
                            lerp(a.vh, b.vh, t)};
}
