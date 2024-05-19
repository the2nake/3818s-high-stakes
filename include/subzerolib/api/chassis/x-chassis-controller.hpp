#pragma once

#include "chassis-controller.hpp"
#include "pros/abstract_motor.hpp"
#include <memory>

class XChassisController : public ChassisController {
public:
  void move(double x, double y, double r) override;

private:
  XChassisController() {}

  std::unique_ptr<pros::AbstractMotor> front_left;
  std::unique_ptr<pros::AbstractMotor> front_right;
  std::unique_ptr<pros::AbstractMotor> back_right;
  std::unique_ptr<pros::AbstractMotor> back_left;

public:
  class XChassisControllerBuilder {
  public:
    XChassisControllerBuilder &
    with_motors(std::unique_ptr<pros::AbstractMotor> ifront_left,
                std::unique_ptr<pros::AbstractMotor> ifront_right,
                std::unique_ptr<pros::AbstractMotor> iback_right,
                std::unique_ptr<pros::AbstractMotor> iback_left);

    std::shared_ptr<XChassisController> build();

  private:
    bool failed = false;
    
    std::unique_ptr<pros::AbstractMotor> bfront_left = nullptr;
    std::unique_ptr<pros::AbstractMotor> bfront_right = nullptr;
    std::unique_ptr<pros::AbstractMotor> bback_right = nullptr;
    std::unique_ptr<pros::AbstractMotor> bback_left = nullptr;
  };
};