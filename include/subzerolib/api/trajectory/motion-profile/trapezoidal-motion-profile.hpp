#pragma once

#include "subzerolib/api/trajectory/motion-profile/linear-motion-profile.hpp"
#include <cmath>
#include <stdio.h>
#include <vector>

class TrapezoidalMotionProfile : public LinearMotionProfile {
public:
  /// @brief creates a trapezoidal motion profile
  /// @param imax_vel the maximum velocity
  /// @param imax_accel the maximum acceleration
  /// @param imax_decel the maximum deceleration
  /// @returns the motion profile object
  TrapezoidalMotionProfile(double imax_vel,
                           double imax_accel,
                           double imax_decel = std::nan(""));

  /// @brief sets the steps between generated points
  /// @param step_length distance covered by each step
  void set_resolution(double step_length) override;

  /// @brief generate the motion profile for a given distance
  /// @param distance final distance covered
  void generate(double distance) override;

  /// @brief queries a point by time
  /// @param time the time corresponding to the point
  profile_point_s get_point_at_time(double time) override;

  /// @brief queries a point by distance
  /// @param distance the distance corresponding to the point
  profile_point_s get_point_at_distance(double distance) override;

  double get_max_vel() override { return max_vel; }
  double get_max_accel() override { return max_accel; }
  double get_max_decel() override { return max_decel; }

  /// @brief prints out the motion profile's points
  void print() {
    for (auto &point : points) {
      printf("t: %5.2f x: %5.2f v: %5.2f\n", point.t, point.x, point.v);
    }
    fflush(stdout);
  }

private:
  const double max_vel;
  const double max_accel;
  const double max_decel;

  double resolution = 0.05; // distance
  std::vector<profile_point_s> points;
};
