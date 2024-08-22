#include "subzerolib/api/geometry/trajectory-point.hpp"

double trajectory_point_s::v() const { return std::hypot(vx, vy); }

std::string trajectory_point_s::to_string() const {
  return "tp(t=" + std::to_string(t) + " s=" + std::to_string(s) + " " +
         point_s::to_string() + " v(" + std::to_string(vx) + ", " +
         std::to_string(vy) + ", " + std::to_string(vh) + ")";
};

bool operator==(trajectory_point_s &a, trajectory_point_s &b) {
  return a.x == b.x && a.y == b.y && a.s == b.s && a.t == b.t && a.x == b.x &&
         a.y == b.y && a.h == b.h && a.vx == b.vx && a.vy == b.vy &&
         a.vh == b.vh;
}

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
