#pragma once

#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/geometry/pose.hpp"

struct encoder_conf_s {
  double offset;
  double travel_per_deg;
};

class Odometry {
public:
  virtual void set_heading(double heading) = 0;
  virtual void set_position(double x, double y) = 0;

  virtual pose_s get_pose() = 0;
  virtual point_s get_vel() = 0;

  virtual void update() = 0;

protected:
  Odometry() {}
};
