#include "subzerolib/api/control/pure-pursuit.hpp"
#include "subzerolib/api/geometry/circle.hpp"
#include "subzerolib/api/geometry/pose.hpp"
#include <memory>

PurePursuitController::PurePursuitController(
    std::shared_ptr<ChassisController> ichassis, std::shared_ptr<Odometry> iodom,
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

  pose_s carrot = waypoints.front();

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

    // select carrot pose, heading is lerped if intersection is used
    if (waypoints.size() == 1) {
      carrot = waypoints[0];
    } else {
      segment_s segment{waypoints[0].point(), waypoints[1].point()};
      auto intersections = seek_circle.intersections(segment);
      if (intersections.size() == 0) {
        carrot = waypoints[0];
      } else {
        carrot = pose_s{
            intersections[0],
            lerp(waypoints[0].heading(), waypoints[1].heading(),
                 intersections[0].dist(segment.start) / segment.length())};
        if (carrot.dist(segment.end) > intersections.back().dist(segment.end)) {
          carrot = pose_s{intersections.back(),
                          lerp(waypoints[0].heading(), waypoints[1].heading(),
                               intersections.back().dist(segment.start) /
                                   segment.length())};
        }
      }
    }

    // use chassis controller implementation
    chassis->approach_pose(carrot);

    // check if settled
    if (exit_condition->is_met()) {
      break;
    }

    prev_pose = curr_pose;
  }

  chassis->brake();
  while (!mutex.take(5)) {
    pros::delay(1);
  }
  motion_complete = true;
  mutex.give();
}