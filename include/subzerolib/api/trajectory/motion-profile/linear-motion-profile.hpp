#pragma once

#include <map>
#include <vector>

struct profile_point_s {
  double t;
  double x;
  double v;
};

class LinearMotionProfile {
public:
  virtual void set_time_resolution(double ms) = 0;
  virtual void generate(double distance) = 0;

  virtual profile_point_s get_point_at_time(double time) = 0;
  virtual profile_point_s get_point_at_distance(double distance) = 0;

protected:
  LinearMotionProfile() {}
};