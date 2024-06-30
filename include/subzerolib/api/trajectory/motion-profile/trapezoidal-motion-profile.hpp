#include "subzerolib/api/trajectory/motion-profile/linear-motion-profile.hpp"
#include <stdio.h>

class TrapezoidalMotionProfile : LinearMotionProfile {
public:
  TrapezoidalMotionProfile(double imax_vel, double imax_accel,
                           double imax_decel);

  void set_resolution(double dist) override;
  void generate(double distance) override;

  profile_point_s get_point_at_time(double time) override {
    return {0, 0, 0}; // FIXME: lerp
  }
  profile_point_s get_point_at_distance(double distance) override {
    return {0, 0, 0}; // FIXME: lerp
  }

  void print() {
    for (auto &point : points) {
      printf("%f, %f, %f\n", point.t, point.x, point.v);
    }
    fflush(stdout);
  }

private:
  const double max_vel;
  const double max_accel;
  const double max_decel;

  double resolution = 0.05; // ms
  std::vector<profile_point_s> points;
};