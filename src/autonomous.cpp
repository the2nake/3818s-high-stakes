#include "devices.hpp"
#include "main.h"
#include "subzerolib/api/spline/catmull-rom.hpp"
#include "subzerolib/api/trajectory/motion-profile/linear-motion-profile.hpp"
#include "subzerolib/api/trajectory/motion-profile/trapezoidal-motion-profile.hpp"
#include "subzerolib/api/trajectory/spline-trajectory.hpp"

/*
#include <memory>
#include <random>
*/

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

  // controller->move_to_pose({0.3, 0.3, 270});
  // controller->move_to_pose({-0.6, 0.5, 315});

  /*
  std::random_device r;
  std::default_random_engine e1(r());
  std::uniform_int_distribution<int> uniform_int(-2, 2);
  std::uniform_real_distribution<double> uniform_float(0.0, 360.0);
  for (int i = 0; i < 16; ++i) {
    double x = 0.25 * uniform_int(e1);
    double y = 0.25 * uniform_int(e1);
    double h = uniform_float(e1);
    controller->move_to_pose({x, y, h});
  }
  controller->move_to_pose({0.0, 0.0, 0.0});
  */

  std::shared_ptr<ExitCondition<double>> cond{
      new ExitCondition<double>{{0, 0.02}, 200}
  };

  std::vector<pose_s> ctrl = {
      pose_s{  0.0,  0.0,   0.0},
      pose_s{  0.4,  0.6,  45.0},
      pose_s{ -0.2,  0.6,  60.0},
      pose_s{-0.75, 0.75, -45.0}
  };
  std::shared_ptr<CatmullRomSpline> spline{new CatmullRomSpline{ctrl}};
  spline.pad_velocity({0.5, 0.5}, {-0.25, 0.25});

  std::shared_ptr<TrapezoidalMotionProfile> profile{
      new TrapezoidalMotionProfile{chassis->get_max_vel(), 5}
  };

  auto traj =
      SplineTrajectory::Builder(SplineTrajectory::heading_mode_e::pose, 400)
          .with_spline(spline, ctrl)
          .with_chassis(chassis)
          .with_motion_profile(profile)
          .build();

  /*
  PurePursuitController pp{controller, odom, std::move(cond)};
  auto spline_points = spline.sample_coordinates(200);
  std::vector<pose_s> waypoints = interpolate_heading(spline_points, ctrl);
  pp.follow(waypoints, 0.4);
  */
}
