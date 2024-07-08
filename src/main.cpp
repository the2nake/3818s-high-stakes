#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/screen.hpp"
#include "subzerolib/api.hpp"
#include "subzerolib/api/chassis/star-chassis.hpp"
#include "subzerolib/api/control/holo-chassis-pid.hpp"
#include "subzerolib/api/control/pure-pursuit.hpp"
#include "subzerolib/api/odometry/imu_odometry.hpp"
#include "subzerolib/api/odometry/odometry.hpp"
#include "subzerolib/api/sensors/abstract_encoder.hpp"
#include "subzerolib/api/sensors/abstract_gyro.hpp"
#include "subzerolib/api/spline/catmull-rom.hpp"
#include "subzerolib/api/util/math.hpp"
#include <algorithm>
#include <memory>

namespace saturnine {
bool running = true;
};

// TODO: test PID on the arm

std::unique_ptr<pros::Motor> fl(new pros::Motor(-1, pros::v5::MotorGears::green,
                                                pros::v5::MotorUnits::deg));
std::unique_ptr<pros::Motor> fr(new pros::Motor(10, pros::v5::MotorGears::green,
                                                pros::v5::MotorUnits::deg));
std::unique_ptr<pros::Motor> ml(new pros::Motor(-12,
                                                pros::v5::MotorGears::green,
                                                pros::v5::MotorUnits::deg));
std::unique_ptr<pros::Motor> mr(new pros::Motor(19, pros::v5::MotorGears::green,
                                                pros::v5::MotorUnits::deg));
std::unique_ptr<pros::Motor> br(new pros::Motor(20, pros::v5::MotorGears::green,
                                                pros::v5::MotorUnits::deg));
std::unique_ptr<pros::Motor> bl(new pros::Motor(-11,
                                                pros::v5::MotorGears::green,
                                                pros::v5::MotorUnits::deg));
std::shared_ptr<StarChassis> chassis = nullptr;
std::shared_ptr<AbstractGyro> imu{new AbstractImuGyro(9)};

// TODO: make ports.h
std::shared_ptr<AbstractEncoder> odom_x{new AbstractRotationEncoder(6, true)};
std::shared_ptr<AbstractEncoder> odom_y{new AbstractRotationEncoder(7, true)};
std::shared_ptr<Odometry> odom = nullptr;

void initialize() {
  auto imus = pros::Imu::get_all_devices();
  for (auto device : imus) {
    device.reset(true);
  }
  pros::delay(250);

  pros::screen::print(pros::E_TEXT_MEDIUM, 10, "%d", imus.size());

  chassis =
      StarChassis::Builder()
          .with_motors(StarChassis::motor_position_e::front_left, std::move(fl))
          .with_motors(StarChassis::motor_position_e::front_right,
                       std::move(fr))
          .with_motors(StarChassis::motor_position_e::boost_left, std::move(ml))
          .with_motors(StarChassis::motor_position_e::boost_right,
                       std::move(mr))
          .with_motors(StarChassis::motor_position_e::back_left, std::move(bl))
          .with_motors(StarChassis::motor_position_e::back_right, std::move(br))
          .with_geometry(0.35, 0.37)
          .with_rot_pref(0.3)
          .build();

  odom =
      ImuOdometry::Builder()
          .with_gyro(imu)
          .with_x_enc(odom_x, Odometry::encoder_conf_s(-0.045, 0.160 / 360.0))
          .with_y_enc(odom_y, Odometry::encoder_conf_s(0.09, 0.160 / 360.0))
          .build();
  odom->auto_update(10);

  pros::Task graphing_task{
      [&] {
        std::vector<point_s> past_points;
        while (true) {
          past_points.push_back(odom->get_pose().point());
          if (past_points.size() > 200) {
            past_points.erase(past_points.begin());
          }

          pros::screen::set_pen(pros::Color::black);
          pros::screen::fill_rect(24, 24, 124, 124);
          pros::screen::set_pen(pros::Color::white);
          pros::screen::draw_line(24, 74, 124, 74); // x axis
          pros::screen::draw_line(74, 24, 74, 124); // y axis

          for (auto &point : past_points) {
            pros::screen::draw_circle(74 + 50 * point.x, 74 - 50 * point.y, 2);
          }
          pros::delay(10);
        }
      },
      "graphing task"};
}

void disabled() {}

void competition_initialize() {}

// TODO: write the rest of the test code

void autonomous() {

  std::shared_ptr<HoloChassisPID> controller =
      HoloChassisPID::Builder()
          .with_chassis(chassis)
          .with_odom(odom)
          .with_pid(HoloChassisPID::pid_dimension_e::x, 1.8, 0.0, 0)
          .with_pid(HoloChassisPID::pid_dimension_e::y, 1.8, 0.0, 0)
          .with_pid(HoloChassisPID::pid_dimension_e::r, 0.01, 0.0, 0)
          .build();
  std::unique_ptr<ExitCondition<double>> cond(
      new ExitCondition<double>({0, 0.02}, 400));
  const pose_s target{0.3, 0.3, 270};
  odom->set_position(0.0, 0.0);
  odom->set_heading(0.0);

  // cond->auto_update([&]() -> double { return odom->get_pose().dist(target);
  // },
  //                   10);
  while (!cond->is_met()) {
    controller->approach_pose(target);
    pros::delay(10);
  }
  /*
  // TODO: tune integral
  PurePursuitController pp(controller, odom, std::move(cond));
  std::vector<pose_s> ctrl = {
      {0.0, 0.0, 0.0}, {0.4, 0.6, 45.0}, {-0.2, 0.6, 60.0}, {-1.0, 1.0, -45.0}};
  CatmullRomSpline spline(ctrl);
  spline.pad_velocity({0.5, 0.5}, {-0.25, 0.25});
  auto spline_points = spline.sample(200);
  std::vector<pose_s> waypoints(spline_points.size());
  transform(spline_points.begin(), spline_points.end(), waypoints.begin(),
            [](point_s point) -> pose_s { return pose_s{point}; });

  // TODO: fix bug, data abort exception, path following seems to start
  // correctly
  // TODO: rewrite pure pursuit? it's very short
  pp.follow(waypoints, 0.1);
  */
}

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  while (saturnine::running) {
    // Arcade control scheme
    // FIXME: may need adjustments to increase accuracy along diagonals
    // model as a polar radial percentage of a rounded circle
    double x = master.get_analog(ANALOG_RIGHT_X) / 127.0;
    double y = master.get_analog(ANALOG_RIGHT_Y) / 127.0;
    double r = master.get_analog(ANALOG_LEFT_X) / 127.0;
    auto pose = odom->get_pose();

    if (std::isnan(pose.h)) {
      pose.h = 0.0;
    }

    auto vec = rotate_acw(x, y, pose.h);
    chassis->move(vec.x, vec.y, 0.75 * r);

    // TODO: chassis angle correction
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      // test pausing/starting odometry
      if (odom->is_enabled()) {
        odom->set_enabled(false);
      } else {
        odom->set_enabled(true);
      }
    }
    // print odometry output
    // TODO: graph odometry output
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "x: %.2f, y: %.2f, h: %.1f%s",
                        pose.x, pose.y, pose.heading(), "                   ");

    pros::delay(10); // high update rate, as imu data comes in every 10 ms
  }

  // garbage collection, good practice
  // free all memory
  //   no need in XChassis, everything is a smart pointer
  //   no need for imu, it's a smart pointer
  //   no need in ImuOdometry, everything is a smart pointer
  // delete pointers
}