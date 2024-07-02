#include "subzerolib/api/chassis/star-chassis.hpp"
#include "subzerolib/api/util/controls.hpp"
#include "subzerolib/api/util/helper.hpp"
#include "subzerolib/api/util/math.hpp"
#include <cmath>
#include <memory>
#include <pros/abstract_motor.hpp>

void StarChassis::move(double x, double y, double r) {
  clamp_distance<double>(1.0, x, y);
  insert_or_modify(vels, StarChassis::motor_position_e::front_left, {x + y, r});
  insert_or_modify(vels, StarChassis::motor_position_e::front_right,
                   {-x + y, -r});
  insert_or_modify(vels, StarChassis::motor_position_e::boost_left,
                   {y, r * K_SQRT_2 * boost_radius / corner_radius});
  insert_or_modify(vels, StarChassis::motor_position_e::boost_right,
                   {y, -r * K_SQRT_2 * boost_radius / corner_radius});
  insert_or_modify(vels, StarChassis::motor_position_e::back_left,
                   {-x + y, r});
  insert_or_modify(vels, StarChassis::motor_position_e::back_right,
                   {x + y, -r});

  balance_mapped_vels(vels, 1.0, rot_pref);
  move_with_map();
}

void StarChassis::move_with_map() {
  for (auto &pair : vels) {
    position_ptr_map.at(pair.first)->move_voltage(12000 * pair.second.sum());
  }
}

void StarChassis::set_rot_pref(double irot_pref) {
  rot_pref = std::abs(irot_pref);
  clamp(rot_pref, 0.0, 1.0);
}

StarChassis::Builder &
StarChassis::Builder::with_motors(StarChassis::motor_position_e position,
                                  std::unique_ptr<pros::AbstractMotor> motor) {
  if (motor == nullptr) {
    return *this;
  }
  switch (position) {
  case StarChassis::motor_position_e::front_left:
    bfront_left = std::move(motor);
    break;
  case StarChassis::motor_position_e::front_right:
    bfront_right = std::move(motor);
    break;
  case StarChassis::motor_position_e::boost_left:
    bboost_left = std::move(motor);
    break;
  case StarChassis::motor_position_e::boost_right:
    bboost_right = std::move(motor);
    break;
  case StarChassis::motor_position_e::back_left:
    bback_left = std::move(motor);
    break;
  case StarChassis::motor_position_e::back_right:
    bback_right = std::move(motor);
    break;
  }
  return *this;
}

bool StarChassis::Builder::try_copy(
    std::unique_ptr<pros::AbstractMotor> &target,
    std::unique_ptr<pros::AbstractMotor> &origin) {
  if (origin == nullptr) {
    return false;
  } else {
    target = std::move(origin);
    return true;
  }
}

StarChassis::Builder &
StarChassis::Builder::with_geometry(double iboost_radius,
                                    double icorner_radius) {
  bboost_radius = std::abs(iboost_radius);
  bcorner_radius = std::abs(icorner_radius);
  if (std::isnan(bboost_radius / bcorner_radius) || bboost_radius == 0) {
    bboost_radius = 1;
    bcorner_radius = 1;
  }
  return *this;
}

StarChassis::Builder &StarChassis::Builder::with_rot_pref(double rot_pref) {
  rot_pref = std::abs(rot_pref);
  clamp(rot_pref, 0.0, 1.0);
  this->brot_pref = rot_pref;
  return *this;
}

std::shared_ptr<StarChassis> StarChassis::Builder::build() {
  auto chassis = new StarChassis();
  std::vector<bool> results;
  results.push_back(!try_copy(chassis->front_left, bfront_left));
  results.push_back(!try_copy(chassis->front_right, bfront_right));
  results.push_back(!try_copy(chassis->boost_left, bboost_left));
  results.push_back(!try_copy(chassis->boost_right, bboost_right));
  results.push_back(!try_copy(chassis->back_left, bback_left));
  results.push_back(!try_copy(chassis->back_right, bback_right));
  for (auto result : results) {
    if (result) {
      return nullptr;
    }
  }
  chassis->boost_radius = bboost_radius;
  chassis->corner_radius = bcorner_radius;
  chassis->rot_pref = brot_pref;
  chassis->position_ptr_map = {
      {StarChassis::motor_position_e::front_left, chassis->front_left},
      {StarChassis::motor_position_e::front_right, chassis->front_right},
      {StarChassis::motor_position_e::boost_left, chassis->boost_left},
      {StarChassis::motor_position_e::boost_right, chassis->boost_right},
      {StarChassis::motor_position_e::back_left, chassis->back_left},
      {StarChassis::motor_position_e::back_right, chassis->back_right},
  };
  return std::shared_ptr<StarChassis>{chassis};
}