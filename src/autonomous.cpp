#include "main.h"
#include "devices.hpp"

void autonomous() {
  std::shared_ptr<HoloChassisPID> controller =
      HoloChassisPID::Builder()
          .with_chassis(chassis)
          .with_odom(odom)
          .with_pid(HoloChassisPID::pid_dimension_e::x, 3.6, 0, 0.42)
          .with_pid(HoloChassisPID::pid_dimension_e::y, 3.6, 0, 0.42)
          .with_pid(HoloChassisPID::pid_dimension_e::r, 0.015, 0.0, 0.0008)
          .build();
  odom->set_position(0.0, 0.0);
  odom->set_heading(0.0);

  controller->move_to_pose({0.3, 0.3, 270});
  controller->move_to_pose({-0.6, 0.5, 315});

  std::shared_ptr<ExitCondition<double>> cond{
      new ExitCondition<double>{{0, 0.02}, 200}
  };
  PurePursuitController pp{controller, odom, std::move(cond)};

  CatmullRomSpline spline({
      pose_s{  0.0,  0.0,   0.0},
      pose_s{  0.4,  0.6,  45.0},
      pose_s{ -0.2,  0.6,  60.0},
      pose_s{-0.75, 0.75, -45.0}
      });
  spline.pad_velocity({0.5, 0.5}, {-0.25, 0.25});
  auto spline_points = spline.sample_coordinates(200);
  std::vector<pose_s> waypoints(spline_points.size());
  transform(spline_points.begin(),
            spline_points.end(),
            waypoints.begin(),
            [](point_s point) -> pose_s { return pose_s{point, 0.0}; });
  // pp.follow(waypoints, 0.4);
}
