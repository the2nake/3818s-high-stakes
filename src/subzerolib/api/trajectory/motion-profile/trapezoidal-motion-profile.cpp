#include "subzerolib/api/trajectory/motion-profile/trapezoidal-motion-profile.hpp"
#include "linear-motion-profile.hpp"

TrapezoidalMotionProfile::TrapezoidalMotionProfile(double imax_vel,
                                                   double imax_accel,
                                                   double imax_decel)
    : max_vel(imax_vel), max_accel(imax_accel), max_decel(imax_decel) {
  // paranoia
}

void TrapezoidalMotionProfile::set_time_resolution(double ms) {
  resolution = std::max(0.1, std::abs(ms));
}

void TrapezoidalMotionProfile::generate(double distance) {
  points.clear();
  std::vector<profile_point_s> points_from_back;

  // start with distance parameterization
  points.emplace_back(0, 0, 0);
  points_from_back.emplace_back(0, distance, 0);

  profile_point_s front_head{0, 0, 0};
  profile_point_s back_head{0, 0, 0};
  
  // NOTE: negate if gone
  while (back_head.x - front_head.x > resolution) {
    // move in from both sides
    // propagate accordingly
  }
}
