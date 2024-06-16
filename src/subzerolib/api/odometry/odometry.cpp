#include "subzerolib/api/odometry/odometry.hpp"
#include "pros/rtos.hpp"

/// @brief repeatedly updates odometry
/// @param iconfig the configuration. must be dynamically allocated
void update_odometry_callback_loop(void *iconfig) {
  auto config = (odom_update_conf_s *)iconfig;
  while (true) {
    config->odom->update();
    pros::delay(config->delay);
  }
}

void automatic_update(std::shared_ptr<Odometry> odom, int ms_delay) {
  auto conf = new odom_update_conf_s(odom, ms_delay);
  pros::Task(update_odometry_callback_loop, conf, "odometry update task");
}
