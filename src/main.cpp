#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.hpp"
#include "subzerolib/api.hpp"
#include "subzerolib/api/odometry/imu_odometry.hpp"
#include "subzerolib/api/odometry/odometry.hpp"
#include "subzerolib/api/sensors/abstract_encoder.hpp"
#include "subzerolib/api/sensors/abstract_gyro.hpp"
#include <memory>

namespace restless {
bool running = true;
};

std::unique_ptr<pros::AbstractMotor> fl(
    new pros::Motor(1, pros::v5::MotorGears::green, pros::v5::MotorUnits::deg));
std::unique_ptr<pros::AbstractMotor>
    fr(new pros::Motor(10, pros::v5::MotorGears::green,
                       pros::v5::MotorUnits::deg));
std::unique_ptr<pros::AbstractMotor>
    br(new pros::Motor(20, pros::v5::MotorGears::green,
                       pros::v5::MotorUnits::deg));
std::unique_ptr<pros::AbstractMotor>
    bl(new pros::Motor(11, pros::v5::MotorGears::green,
                       pros::v5::MotorUnits::deg));
std::shared_ptr<XChassis> chassis = nullptr;
// TODO: configure ports
std::shared_ptr<AbstractEncoder> odom_x(new AbstractRotationEncoder(9, false));
std::shared_ptr<AbstractEncoder> odom_y(new AbstractRotationEncoder(10, false));
std::shared_ptr<Odometry> odom = nullptr;
std::shared_ptr<AbstractGyro> imu(new AbstractImuGyro(8));

pros::Task *odom_task;
void initialize() {
  chassis = XChassis::XChassisBuilder()
                .with_motors(std::move(fl), std::move(fr), std::move(br),
                             std::move(bl))
                .build();
  // TODO: configure odometry
  odom = ImuOdometry::ImuOdometryBuilder()
             .with_gyro(imu)
             .with_x_enc(odom_x, Odometry::encoder_conf_s(0, 0))
             .with_y_enc(odom_y, Odometry::encoder_conf_s(0, 0))
             .build();
  odom_task = automatic_update(odom, 10);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  while (restless::running) {
    // Arcade control scheme
    double x = master.get_analog(ANALOG_RIGHT_X) / 127.0;
    double y = master.get_analog(ANALOG_RIGHT_Y) / 127.0;
    double r = master.get_analog(ANALOG_LEFT_X) / 127.0;

    double h = imu->get_heading();
    double rad = -0.01745329251 * h;
    chassis->move(x * cos(rad) + y * sin(rad), y * cos(rad) - x * sin(rad),
                  0.75 * r);
    // chassis->move(x, y, r);
    pros::delay(20); // Run for 20 ms then update
  }

  // garbage collection, good practice
  // free all memory
  //   no need in XChassis, everything is a smart pointer
  //   no need for imu, it's a smart pointer
  //   no need in ImuOdometry, everything is a smart pointer
  // notify to delete the odometry update task
  odom_task->notify();
  // delete pointers
}