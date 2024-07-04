#pragma once

#include "subzerolib/api/chassis/chassis.hpp"
#include "subzerolib/api/util/controls.hpp"

#include "pros/motors.hpp"
#include "pros/abstract_motor.hpp"
#include <map>
#include <memory>

class StarChassis : public Chassis {
public:
  enum class motor_position_e {
    front_left,
    front_right,
    boost_left,
    boost_right,
    back_left,
    back_right
  };
  void move(double x, double y, double r) override;
  void set_rot_pref(double irot_pref = 0.5) override;

private:
  StarChassis() {}
  void move_with_map();

  double boost_radius;
  double corner_radius;
  double rot_pref;

  std::map<StarChassis::motor_position_e, control_components_s> vels = {};
  std::map<StarChassis::motor_position_e,
           std::unique_ptr<pros::AbstractMotor> &>
      position_ptr_map = {};
  std::unique_ptr<pros::AbstractMotor> front_left = nullptr;
  std::unique_ptr<pros::AbstractMotor> front_right = nullptr;
  std::unique_ptr<pros::AbstractMotor> back_left = nullptr;
  std::unique_ptr<pros::AbstractMotor> back_right = nullptr;
  std::unique_ptr<pros::AbstractMotor> boost_left = nullptr;
  std::unique_ptr<pros::AbstractMotor> boost_right = nullptr;

public:
  class Builder {
  public:
    Builder &with_motors(motor_position_e dimension,
                         std::unique_ptr<pros::Motor> motor);
    Builder &with_motors(motor_position_e dimension,
                         std::unique_ptr<pros::AbstractMotor> motor);
    Builder &with_geometry(double iboost_radius, double icorner_radius);
    Builder &with_rot_pref(double rot_pref = 0.5);

    std::shared_ptr<StarChassis> build();

  private:
    bool try_copy(std::unique_ptr<pros::AbstractMotor> &target,
                  std::unique_ptr<pros::AbstractMotor> &origin);
    double bboost_radius;
    double bcorner_radius;
    double brot_pref;
    std::unique_ptr<pros::AbstractMotor> bfront_left = nullptr;
    std::unique_ptr<pros::AbstractMotor> bfront_right = nullptr;
    std::unique_ptr<pros::AbstractMotor> bback_left = nullptr;
    std::unique_ptr<pros::AbstractMotor> bback_right = nullptr;
    std::unique_ptr<pros::AbstractMotor> bboost_left = nullptr;
    std::unique_ptr<pros::AbstractMotor> bboost_right = nullptr;
  };
};