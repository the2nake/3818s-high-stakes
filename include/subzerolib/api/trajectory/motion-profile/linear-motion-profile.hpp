#pragma once

struct profile_point_s {
  profile_point_s(double it, double ix, double iv) : t(it), x(ix), v(iv) {}

  double t;
  double x;
  double v;
};

class LinearMotionProfile {
public:
  virtual void set_resolution(double dist) = 0;
  virtual void generate(double distance) = 0;

  virtual profile_point_s get_point_at_time(double time) = 0;
  virtual profile_point_s get_point_at_distance(double distance) = 0;

protected:
  LinearMotionProfile() {}
};