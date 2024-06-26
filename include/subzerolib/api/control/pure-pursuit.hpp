#pragma once

#include "pros/rtos.hpp"
#include "subzerolib/api/control/chassis-controller.hpp"
#include "subzerolib/api/logic/exit-condition.hpp"
#include "subzerolib/api/geometry/pose.hpp"
#include "subzerolib/api/odometry/odometry.hpp"
#include <atomic>
#include <memory>
#include <vector>

class PurePursuitController {
public:
  PurePursuitController(std::shared_ptr<ChassisController> ichassis,
                        std::shared_ptr<Odometry> iodom,
                        std::unique_ptr<ExitCondition<double>> ipos_exit_condition);
  /// @brief follows the path described by the linear spline connecting the
  /// waypoints
  ///
  /// @param waypoints a vector of waypoints
  /// @param lookahead range for pure-pursuit smoothening
  /// @param ms_timeout maximum controller run time
  /// @param resolution number of physics steps per iteration, min 1
  void follow(std::vector<pose_s> iwaypoints, double lookahead,
              int ms_timeout = 5000, int iresolution = 1);
  // TODO: PurePursuitController::follow_async
  void stop();

  bool is_complete() { return motion_complete.load(); }

private:
  void select_carrot(pose_s pose, double lookahead, pose_s &carrot);
  std::vector<pose_s> waypoints;
  int resolution = 1;

  std::shared_ptr<ChassisController> chassis;
  std::shared_ptr<Odometry> odom;
  std::unique_ptr<ExitCondition<double>> pos_exit_condition;

  std::atomic<bool> motion_complete = true;
  pros::Mutex mutex;
};