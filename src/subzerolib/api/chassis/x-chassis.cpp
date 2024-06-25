#include "subzerolib/api/chassis/x-chassis.hpp"
#include "subzerolib/api/chassis/chassis.hpp"
#include "subzerolib/api/util/controls.hpp"
#include "subzerolib/api/util/math.hpp"
#include <cmath>

void XChassis::move(double x, double y, double r) {
  clamp_distance(1.0, x, y);

  std::vector<control_components_s> vs = {
      {x + y, r}, {-x + y, -r}, {x + y, -r}, {-x + y, r}};

  balance_vels(vs, 1.0, rot_pref);

  front_left->move_voltage(12000 * (vs[0].sum()));
  front_right->move_voltage(12000 * (vs[1].sum()));
  back_right->move_voltage(12000 * (vs[2].sum()));
  back_left->move_voltage(12000 * (vs[3].sum()));
}

XChassis::XChassisBuilder &XChassis::XChassisBuilder::with_motors(
    std::unique_ptr<pros::AbstractMotor> ifront_left,
    std::unique_ptr<pros::AbstractMotor> ifront_right,
    std::unique_ptr<pros::AbstractMotor> iback_right,
    std::unique_ptr<pros::AbstractMotor> iback_left) {
  this->bfront_left = std::move(ifront_left);
  this->bfront_right = std::move(ifront_right);
  this->bback_right = std::move(iback_right);
  this->bback_left = std::move(iback_left);
  return *this;
}

std::shared_ptr<XChassis> XChassis::XChassisBuilder::build() {
  std::shared_ptr<XChassis> chassis(new XChassis());

  chassis->rot_pref = brot_pref;
  chassis->front_left = std::move(bfront_left);
  chassis->front_right = std::move(bfront_right);
  chassis->back_right = std::move(bback_right);
  chassis->back_left = std::move(bback_left);

  return chassis;
}