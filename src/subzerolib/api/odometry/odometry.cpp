#include "subzerolib/api/odometry/odometry.hpp"
#include "pros/rtos.hpp"

// TODO: change this to notify w/ pause/kill!

/// @brief repeatedly updates odometry
/// @param iconfig the configuration. must be dynamically allocated
void update_odometry_callback_loop(void *iconfig) {
  auto config = (odom_update_conf_s *)iconfig;
  uint32_t prev_time = pros::millis();
  bool paused = false;
  bool odom_running = true;
  while (odom_running) {
    if (pros::Task::notify_take(true, 0)) {
      switch (config->signal->load()) {
      case 1:
        paused = false;
        break;
      case 0:
        paused = true;
        break;
      case -1:
        // kill
        odom_running = false;
        break;
      default:
        paused = false;
      }
    }
    if (odom_running && !paused) {
      config->odom->update();
    }
    pros::Task::delay_until(&prev_time, config->delay);
  }
  // free the configuration data
  delete (odom_update_conf_s *)iconfig;
}

task_updater_conf_s automatic_update(std::shared_ptr<Odometry> odom,
                                     int ms_delay) {
  // default starting
  auto signal = new std::atomic<int>(1);
  auto conf = new odom_update_conf_s(signal, odom, ms_delay);
  auto update_task = new pros::Task{update_odometry_callback_loop, conf,
                                    "odometry update task"};
  return task_updater_conf_s(update_task, signal);
}