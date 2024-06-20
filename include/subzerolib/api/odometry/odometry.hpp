#pragma once

#include "pros/rtos.hpp"
#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/geometry/pose.hpp"
#include <memory>

class Odometry {
public:
  struct encoder_conf_s {
    encoder_conf_s(double ioffset, double itravel)
        : offset(ioffset), travel_per_deg(itravel) {}
    double offset; // right is +x, up is +y
    double travel_per_deg;
  };

  virtual void set_heading(double heading) = 0;
  virtual void set_position(double x, double y) = 0;

  virtual pose_s get_pose() = 0;
  virtual point_s get_vel() = 0;

  virtual void update() = 0;

  virtual void set_enabled(bool) = 0;

protected:
  Odometry() {}
};

struct odom_update_conf_s {
  std::shared_ptr<Odometry> odom;
  int delay;
};

void update_odometry_callback_loop(void *params);
pros::Task *automatic_update(std::shared_ptr<Odometry> odom, int ms_delay);
