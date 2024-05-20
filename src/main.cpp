#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.hpp"
#include "subzerolib/api.hpp"
#include <memory>

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
pros::Imu imu(8);

void initialize() {
  chassis = XChassis::XChassisBuilder()
                .with_motors(std::move(fl), std::move(fr), std::move(br),
                             std::move(bl))
                .build();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  while (true) {
    // Arcade control scheme
    double x = master.get_analog(ANALOG_RIGHT_X) / 127.0;
    double y = master.get_analog(ANALOG_RIGHT_Y) / 127.0;
    double r = master.get_analog(ANALOG_LEFT_X) / 127.0;

    double h = imu.get_heading();
    double rad = -0.01745329251 * h;
    chassis->move(x * cos(rad) + y * sin(rad), y * cos(rad) - x * sin(rad),
                  0.75 * r);
    // chassis->move(x, y, r);
    pros::delay(20); // Run for 20 ms then update
  }
}