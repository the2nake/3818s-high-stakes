#include "main.h"
#include "subzerolib/api/chassis/star-chassis.hpp"
#include "subzerolib/api/control/holo-chassis-pid.hpp"
#include "subzerolib/api/control/pure-pursuit.hpp"
#include "subzerolib/api/odometry/imu_odometry.hpp"
#include "subzerolib/api/odometry/odometry.hpp"
#include "subzerolib/api/sensors/abstract_encoder.hpp"
#include "subzerolib/api/sensors/abstract_gyro.hpp"
#include "subzerolib/api/spline/catmull-rom.hpp"
#include "subzerolib/api/util/math.hpp"
#include "subzerolib/api.hpp"
#include <memory>
#include <pros/abstract_motor.hpp>

namespace restless {
bool running = true;
};

std::unique_ptr<pros::AbstractMotor> fl(
    new pros::Motor(-1, pros::v5::MotorGears::green, pros::v5::MotorUnits::deg));
std::unique_ptr<pros::AbstractMotor>
    fr(new pros::Motor(10, pros::v5::MotorGears::green,
                       pros::v5::MotorUnits::deg));
std::unique_ptr<pros::AbstractMotor>
    ml(new pros::Motor(-12, pros::v5::MotorGears::green,
                       pros::v5::MotorUnits::deg));
std::unique_ptr<pros::AbstractMotor>
    mr(new pros::Motor(19, pros::v5::MotorGears::green,
                       pros::v5::MotorUnits::deg));
std::unique_ptr<pros::AbstractMotor>
    br(new pros::Motor(20, pros::v5::MotorGears::green,
                       pros::v5::MotorUnits::deg));
std::unique_ptr<pros::AbstractMotor>
    bl(new pros::Motor(-11, pros::v5::MotorGears::green,
                       pros::v5::MotorUnits::deg));
std::shared_ptr<StarChassis> chassis = nullptr;
std::shared_ptr<AbstractGyro> imu(new AbstractImuGyro(8));

// TODO: make ports.h
std::shared_ptr<AbstractEncoder> odom_x(new AbstractRotationEncoder(6, true));
std::shared_ptr<AbstractEncoder> odom_y(new AbstractRotationEncoder(7, true));
std::shared_ptr<Odometry> odom = nullptr;

void initialize() {
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
          .with_x_enc(odom_x, Odometry::encoder_conf_s(-0.045, 160.0 / 360.0))
          .with_y_enc(odom_y, Odometry::encoder_conf_s(0.09, 160.0 / 360.0))
          .build();
  odom->auto_update(10);
}

void disabled() {}

void competition_initialize() {}

// TODO: write the rest of the test code

void autonomous() {
  std::unique_ptr<ExitCondition<double>> cond(
      new ExitCondition<double>({0, 10}, 400));
  // TODO: tune
  std::shared_ptr<HoloChassisPID> controller =
      HoloChassisPID::Builder()
          .with_chassis(chassis)
          .with_odom(odom)
          .with_pid(HoloChassisPID::pid_dimension_e::x, 100.0, 0.0, 0.0)
          .with_pid(HoloChassisPID::pid_dimension_e::y, 100.0, 0.0, 0.0)
          .with_pid(HoloChassisPID::pid_dimension_e::r, 100.0, 0.0, 0.0)
          .build();
  PurePursuitController pp(controller, odom, std::move(cond));
  std::vector<pose_s> control_points = {}; // TODO: invent
  CatmullRomSpline spline(control_points);
  // TODO: create full trajectory generation
  // test: pp.follow(std::vector<pose_s> iwaypoints, double lookahead);
}

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  while (restless::running) {
    // Arcade control scheme
    // FIXME: may need adjustments to increase accuracy along diagonals
    double x = master.get_analog(ANALOG_RIGHT_X) / 127.0;
    double y = master.get_analog(ANALOG_RIGHT_Y) / 127.0;
    double r = master.get_analog(ANALOG_LEFT_X) / 127.0;
    auto pose = odom->get_pose();

    auto vec = rotate_acw(x, y, pose.h);
    chassis->move(vec.x, vec.y, 0.75 * r);
    // chassis->move(x, y, r);

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      // test pausing/starting odometry
      if (odom->is_enabled()) {
        odom->set_enabled(false);
      } else {
        odom->set_enabled(true);
      }
    }
    // print odometry output
    pros::screen::print(pros::E_TEXT_MEDIUM, 0, "x: %f, y: %f, h: %f", pose.x,
                        pose.y, pose.heading());

    pros::delay(20); // Run for 20 ms then update
  }

  // garbage collection, good practice
  // free all memory
  //   no need in XChassis, everything is a smart pointer
  //   no need for imu, it's a smart pointer
  //   no need in ImuOdometry, everything is a smart pointer
  // notify to delete the odometry update task
  odom->stop_updating();
  // delete pointers
}