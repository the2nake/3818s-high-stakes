#pragma once

#include "chassis.hpp"
#include "pros/abstract_motor.hpp"
#include <memory>

class XChassis : public Chassis {
public:
  void move(double x, double y, double r) override;

private:
  XChassis() {}

  std::unique_ptr<pros::AbstractMotor> front_left;
  std::unique_ptr<pros::AbstractMotor> front_right;
  std::unique_ptr<pros::AbstractMotor> back_right;
  std::unique_ptr<pros::AbstractMotor> back_left;

public:
  class XChassisBuilder {
  public:
    XChassisBuilder &
    with_motors(std::unique_ptr<pros::AbstractMotor> ifront_left,
                std::unique_ptr<pros::AbstractMotor> ifront_right,
                std::unique_ptr<pros::AbstractMotor> iback_right,
                std::unique_ptr<pros::AbstractMotor> iback_left);

    std::shared_ptr<XChassis> build();

  private:
    bool failed = false;
    
    std::unique_ptr<pros::AbstractMotor> bfront_left = nullptr;
    std::unique_ptr<pros::AbstractMotor> bfront_right = nullptr;
    std::unique_ptr<pros::AbstractMotor> bback_right = nullptr;
    std::unique_ptr<pros::AbstractMotor> bback_left = nullptr;
  };
};