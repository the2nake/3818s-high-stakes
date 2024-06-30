#include "subzerolib/api/trajectory/motion-profile/linear-motion-profile.hpp"

class TrapezoidalMotionProfile : LinearMotionProfile {
public:
  TrapezoidalMotionProfile(double imax_vel, double imax_accel,
                           double imax_decel);

  void set_time_resolution(double ms) override;
  void generate(double distance) override;

  profile_point_s get_point_at_time(double time) override;
  profile_point_s get_point_at_distance(double distance) override;

private:
  const double max_vel;
  const double max_accel;
  const double max_decel;

  double resolution = 5; // ms
  std::vector<profile_point_s> points;
};