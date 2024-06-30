#include "subzerolib/api/trajectory/motion-profile/trapezoidal-motion-profile.hpp"

TrapezoidalMotionProfile::TrapezoidalMotionProfile(double imax_vel,
                                                   double imax_accel,
                                                   double imax_decel)
    : max_vel(imax_vel), max_accel(imax_accel), max_decel(imax_decel) {
  // paranoia
}

void TrapezoidalMotionProfile::generate(double distance) {
  points.clear();
  
}
