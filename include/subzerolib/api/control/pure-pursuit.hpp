#pragma once

#include "pros/rtos.hpp"
#include "subzerolib/api/chassis/chassis.hpp"
#include "subzerolib/api/control/exit-condition.hpp"
#include "subzerolib/api/geometry/pose.hpp"
#include "subzerolib/api/odometry/odometry.hpp"
#include <atomic>
#include <memory>
#include <vector>

class PurePursuitController {
public:
  PurePursuitController(std::shared_ptr<Chassis> ichassis,
                        std::shared_ptr<Odometry> iodom,
                        std::unique_ptr<ExitCondition> iexit_condition);

  // TODO: redo exit condition abstraction. need to have some way to pass
  // target position. lemlib?

  /// @brief follows the path described by the linear spline connecting the
  /// waypoints
  ///
  /// @param waypoints a vector of waypoints
  /// @param lookahead range for pure-pursuit smoothening
  /// @param ms_timeout maximum controller run time
  void follow(std::vector<pose_s> waypoints, double lookahead,
              int ms_timeout = 5000);
  void stop();

  bool is_complete() { return motion_complete.load(); }

private:
  std::shared_ptr<Chassis> chassis;
  std::shared_ptr<Odometry> odom;
  std::unique_ptr<ExitCondition> exit_condition;

  std::atomic<bool> motion_complete = true;
  pros::Mutex mutex;
};