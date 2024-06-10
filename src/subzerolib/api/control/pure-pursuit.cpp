#include "subzerolib/api/control/pure-pursuit.hpp"
#include "subzerolib/api/geometry/circle.hpp"
#include "subzerolib/api/geometry/point.hpp"
#include <memory>

PurePursuitController::PurePursuitController(
    std::shared_ptr<Chassis> ichassis, std::shared_ptr<Odometry> iodom,
    std::unique_ptr<ExitCondition> iexit_condition)
    : chassis(std::move(ichassis)), odom(std::move(iodom)),
      exit_condition(std::move(iexit_condition)) {}

void PurePursuitController::follow(std::vector<pose_s> waypoints,
                                   double lookahead, int ms_timeout) {

  // TODO: impl resolution
  while (!mutex.take(5)) {
    pros::delay(1);
  }
  motion_complete = false;
  mutex.give();

  point_s carrot = waypoints.front().point();

  pose_s prev_pose = odom->get_pose();
  pose_s curr_pose = odom->get_pose();
  circle_s seek_circle(curr_pose.x, curr_pose.y, lookahead);

  uint32_t start = pros::millis();

  for (uint32_t duration = 0; duration < ms_timeout;
       duration = pros::millis() - start) {
    curr_pose = odom->get_pose();
    seek_circle = circle_s(curr_pose.x, curr_pose.y, lookahead);

    while (waypoints.size() > 1 && seek_circle.contains(waypoints[1].point())) {
      waypoints.erase(waypoints.begin());
    }

    // select carrot point
    if (waypoints.size() == 1) {
      carrot = waypoints[0].point();
    } else {
      segment_s segment{waypoints[0].point(), waypoints[1].point()};
      auto intersections = seek_circle.intersections(segment);
      if (intersections.size() == 0) {
        carrot = waypoints[0].point();
      } else {
        carrot = intersections[0];
        if (carrot.dist(segment.end) > intersections.back().dist(segment.end)) {
          carrot = intersections.back();
        }
      }
    }

    // TODO: move to the point

    // check if settled
    if (exit_condition->is_met()) {
      break;
    }

    prev_pose = curr_pose;
  }

  chassis->move(0, 0, 0);
  while (!mutex.take(5)) {
    pros::delay(1);
  }
  motion_complete = true;
  mutex.give();
}