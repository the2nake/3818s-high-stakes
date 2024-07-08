#include "subzerolib/api/control/pure-pursuit.hpp"
#include "subzerolib/api/geometry/circle.hpp"
#include "subzerolib/api/geometry/pose.hpp"
#include "subzerolib/api/util/auto-updater.hpp"

#include <memory>

// TODO: pure pursuit untested

PurePursuitController::PurePursuitController(
    std::shared_ptr<ChassisController> ichassis,
    std::shared_ptr<Odometry> iodom,
    std::unique_ptr<ExitCondition<double>> ipos_exit_condition)
    : chassis(std::move(ichassis)), odom(std::move(iodom)),
      pos_exit_condition(std::move(ipos_exit_condition)) {}

void PurePursuitController::follow(std::vector<pose_s> iwaypoints,
                                   double lookahead, int ms_timeout,
                                   int iresolution) {
  waypoints = iwaypoints;
  resolution = std::max(1, iresolution);

  while (!mutex.take(5)) {
    pros::delay(1);
  }
  motion_complete = false;
  mutex.give();

  pose_s carrot = waypoints.front();

  pose_s prev_pose = odom->get_pose();
  pose_s curr_pose = odom->get_pose();
  circle_s seek_circle(curr_pose, lookahead);

  uint32_t start = pros::millis();
  // TODO: test if this works without error
  AutoUpdater<double> updater(
      [&](double val) { pos_exit_condition->update(val); },
      [&]() -> double { return odom->get_pose().dist(waypoints.back()); });
  updater.start(10);

  for (uint32_t duration = 0; duration < ms_timeout;
       duration = pros::millis() - start) {
    curr_pose = odom->get_pose();

    for (int i = 0; i < resolution; ++i) {
      auto check_pose = lerp(prev_pose, curr_pose, i * 1.0 / resolution);
      seek_circle = circle_s(check_pose, lookahead);
      while (waypoints.size() > 1 &&
             seek_circle.contains(waypoints[1].point())) {
        waypoints.erase(waypoints.begin());
      }
    }

    select_carrot(curr_pose, lookahead, carrot);

    // use chassis controller implementation
    chassis->approach_pose(carrot);
    if (motion_complete || pos_exit_condition->is_met()) {
      break;
    }

    prev_pose = curr_pose;
    pros::delay(10);
  }

  updater.stop();

  chassis->brake();
  while (!mutex.take(5)) {
    pros::delay(1);
  }
  motion_complete = true;
  mutex.give();
}

void PurePursuitController::select_carrot(pose_s pose, double lookahead,
                                          pose_s &carrot) {
  // select carrot pose, heading is lerped if intersection is used
  if (waypoints.size() == 1) {
    carrot = waypoints[0];
  } else {
    segment_s segment{waypoints[0].point(), waypoints[1].point()};
    circle_s seek_circle = circle_s(pose, lookahead);
    auto intersections = seek_circle.intersections(segment);
    if (intersections.size() == 0) {
      carrot = waypoints[0];
    } else {
      if (carrot.dist(segment.end) > intersections.back().dist(segment.end)) {
        auto prop = intersections.back().dist(segment.start) / segment.length();
        carrot = lerp(waypoints[0], waypoints[1], prop);
        // assert intersections.back() rougheq lerp result
      } else {
        auto prop = intersections[0].dist(segment.start) / segment.length();
        carrot = lerp(waypoints[0], waypoints[1], prop);
        // assert intersections[0] rougheq lerp result
      }
    }
  }
}

void PurePursuitController::stop() {
  motion_complete = true;
  // pos_exit_condition->stop_updating();
}
