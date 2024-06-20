#include "subzerolib/api/odometry/odometry.hpp"
#include "pros/rtos.hpp"

/// @brief repeatedly updates odometry
/// @param iconfig the configuration. must be dynamically allocated
void update_odometry_callback_loop(void *iconfig) {
  auto config = (odom_update_conf_s *)iconfig;
  uint32_t prev_time = pros::millis();
  while (!pros::Task::notify_take(true, 0)) {
    config->odom->update();
    pros::Task::delay_until(&prev_time, config->delay);
  }
  // free the configuration data
  delete (odom_update_conf_s *)iconfig;
}

pros::Task *automatic_update(std::shared_ptr<Odometry> odom, int ms_delay) {
  auto conf = new odom_update_conf_s(odom, ms_delay);
  auto update_task = new pros::Task{update_odometry_callback_loop, conf,
                                    "odometry update task"};
  return update_task;
}
