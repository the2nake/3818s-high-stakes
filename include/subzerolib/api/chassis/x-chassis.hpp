#pragma once

#include "pros/abstract_motor.hpp"
#include "subzerolib/api/chassis/chassis.hpp"
#include "subzerolib/api/util/math.hpp"
#include <memory>

class XChassis : public Chassis {
public:
  void move(double x, double y, double r) override;
  void set_rot_pref(double irot_pref) override {
    rot_pref = std::abs(irot_pref);
    clamp(rot_pref, 0.0, 1.0);
  }

private:
  XChassis() {}

  double rot_pref = 0.5;
  std::unique_ptr<pros::AbstractMotor> front_left;
  std::unique_ptr<pros::AbstractMotor> front_right;
  std::unique_ptr<pros::AbstractMotor> back_right;
  std::unique_ptr<pros::AbstractMotor> back_left;

public:
  class Builder {
  public:
    Builder &with_motors(std::unique_ptr<pros::AbstractMotor> ifront_left,
                         std::unique_ptr<pros::AbstractMotor> ifront_right,
                         std::unique_ptr<pros::AbstractMotor> iback_right,
                         std::unique_ptr<pros::AbstractMotor> iback_left);
    Builder &with_rot_pref(double irot_pref) {
      brot_pref = std::abs(irot_pref);
      clamp(brot_pref, 0.0, 1.0);
      return *this;
    }

    std::shared_ptr<XChassis> build();

  private:
    bool failed = false;
    double brot_pref = 0.5;
    std::unique_ptr<pros::AbstractMotor> bfront_left = nullptr;
    std::unique_ptr<pros::AbstractMotor> bfront_right = nullptr;
    std::unique_ptr<pros::AbstractMotor> bback_right = nullptr;
    std::unique_ptr<pros::AbstractMotor> bback_left = nullptr;
  };
};