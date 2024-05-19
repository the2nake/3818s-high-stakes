#include "subzerolib/api/chassis/x-chassis-controller.hpp"
#include <cmath>

void XChassisController::move(double x, double y, double r) {

  double d = std::hypot(x, y);
  if (std::abs(d) > 1) {
    double sin = y / d;
    double cos = x / d;
    d = std::min(1.0, d);
    y = sin * d;
    x = cos * d;
  }

  double vs[4] = {y + x + r, y - x - r, y + x - r, y - x + r};
  double max = std::abs(vs[0]);
  for (int i = 1; i < 4; i++) {
    if (std::abs(vs[i]) > max) {
      max = std::abs(vs[i]);
    }
  }

  if (max > 1) {
    vs[0] = vs[0] / max;
    vs[1] = vs[1] / max;
    vs[2] = vs[2] / max;
    vs[3] = vs[3] / max;
  }

  front_left->move_voltage(12000 * (vs[0]));
  front_right->move_voltage(12000 * (vs[1]));
  back_right->move_voltage(12000 * (vs[2]));
  back_left->move_voltage(12000 * (vs[3]));
}

XChassisController::XChassisControllerBuilder &
XChassisController::XChassisControllerBuilder::with_motors(
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

std::shared_ptr<XChassisController>
XChassisController::XChassisControllerBuilder::build() {
  std::shared_ptr<XChassisController> chassis(new XChassisController());

  chassis->front_left = std::move(bfront_left);
  chassis->front_right = std::move(bfront_right);
  chassis->back_right = std::move(bback_right);
  chassis->back_left = std::move(bback_left);

  return chassis;
}