#pragma once

#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/geometry/pose.hpp"

class Odometry {
public:
  virtual pose_s get_pose() = 0;
  virtual point_s get_vel() = 0;

  virtual void update() = 0;

protected:
  Odometry() {}
};
