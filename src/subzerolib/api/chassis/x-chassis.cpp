#include "subzerolib/api/chassis/x-chassis.hpp"
#include "subzerolib/api/util/controls.hpp"
#include "subzerolib/api/util/math.hpp"

void XChassis::move(double x, double y, double r) {
  clamp_distance(1.0, x, y);

  std::vector<control_components_s> vs = {
      { x + y,  r},
      {-x + y, -r},
      { x + y, -r},
      {-x + y,  r}
  };

  balance_vels(vs, 1.0, rot_pref);

  front_left->move_voltage(12000 * (vs[0].sum()));
  front_right->move_voltage(12000 * (vs[1].sum()));
  back_right->move_voltage(12000 * (vs[2].sum()));
  back_left->move_voltage(12000 * (vs[3].sum()));
}

std::vector<double> XChassis::get_wheel_vels(double vx, double vy, double ang) {
  double angular_components[] = {
      radius * ang, radius * ang, -radius * ang, -radius * ang};
  double linear_components[] = {(0.5 * K_SQRT_2) * (vx + vy),
                                (0.5 * K_SQRT_2) * (-vx + vy),
                                (0.5 * K_SQRT_2) * (-vx + vy),
                                (0.5 * K_SQRT_2) * (vx + vy)};
  std::vector<double> final_vels(4);
  for (int i = 0; i < final_vels.size(); ++i) {
    final_vels[i] = linear_components[i] + angular_components[i];
  }
  return final_vels;
}

std::vector<double> XChassis::get_wheel_max() {
  // order is lf, lb, rf, rb
  return {0.5 * K_SQRT_2 * lin_vel,
          0.5 * K_SQRT_2 * lin_vel,
          0.5 * K_SQRT_2 * lin_vel,
          0.5 * K_SQRT_2 * lin_vel};
}

XChassis::Builder &XChassis::Builder::with_motors(
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

XChassis::Builder &XChassis::Builder::with_geometry(double iradius) {
  if (!std::isnan(iradius)) {
    bradius = std::abs(iradius);
  }
  return *this;
}

XChassis::Builder &XChassis::Builder::with_vel(double ilin_vel) {
  if (!std::isnan(ilin_vel)) {
    blin_vel = std::abs(ilin_vel);
  }
  return *this;
}

std::shared_ptr<XChassis> XChassis::Builder::build() {
  std::shared_ptr<XChassis> chassis(new XChassis());

  chassis->rot_pref = brot_pref;
  chassis->radius = bradius;
  chassis->lin_vel = blin_vel;

  chassis->front_left = std::move(bfront_left);
  chassis->front_right = std::move(bfront_right);
  chassis->back_right = std::move(bback_right);
  chassis->back_left = std::move(bback_left);

  return chassis;
}
