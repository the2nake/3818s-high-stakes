#pragma once

#include "pros/rtos.hpp"
#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/geometry/pose.hpp"

class Odometry {
public:
  ~Odometry() {
    if (update_task != nullptr) {
      stop_updating();
      pros::delay(20);
      delete update_task;
      update_task = nullptr;
    }
  }

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
  virtual bool is_enabled() = 0;

  void auto_update(int every_ms = 10) {
    update_delay = every_ms;
    update_task = new pros::Task([&, this] { this->auto_update_loop(); },
                                 "subzerolib: odometry update task");
  }
  void stop_updating() { this->update_task->notify(); }

protected:
  Odometry() {}

private:
  int update_delay = 10;
  pros::Task *update_task = nullptr;
  void auto_update_loop() {
    uint32_t prev_time = pros::millis();
    while (pros::Task::notify_take(true, 0)) {
      this->update();
      pros::Task::delay_until(&prev_time, update_delay);
    }
  }
};
