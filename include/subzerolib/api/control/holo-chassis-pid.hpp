#pragma once

#include "subzerolib/api/chassis/chassis.hpp"
#include "subzerolib/api/control/chassis-controller.hpp"
#include "subzerolib/api/control/pid.hpp"
#include "subzerolib/api/odometry/odometry.hpp"
#include <memory>

class HoloChassisPID : public ChassisController {
public:
  enum class pid_dimension_e {
    x,
    y,
    r,
  };

  void approach_pose(pose_s target, double linv = std::nan("")) override;
  void brake() override { chassis->move(0, 0, 0); }

private:
  std::shared_ptr<Chassis> chassis = nullptr;
  std::shared_ptr<Odometry> odom = nullptr;
  std::unique_ptr<PIDF> x_pid = nullptr;
  std::unique_ptr<PIDF> y_pid = nullptr;
  std::unique_ptr<PIDF> r_pid = nullptr;

  HoloChassisPID() {}

public:
  class HoloChassisPIDBuilder {
  public:
    HoloChassisPIDBuilder &with_chassis(std::shared_ptr<Chassis> ichassis);

    HoloChassisPIDBuilder &with_odom(std::shared_ptr<Odometry> iodom);

    HoloChassisPIDBuilder &with_pid(pid_dimension_e dimension, double kp,
                                    double ki, double kd);

    std::shared_ptr<HoloChassisPID> build();

  private:
    std::shared_ptr<Chassis> bchassis = nullptr;
    std::shared_ptr<Odometry> bodom = nullptr;
    std::unique_ptr<PIDF> bx_pid = nullptr;
    std::unique_ptr<PIDF> by_pid = nullptr;
    std::unique_ptr<PIDF> br_pid = nullptr;
  };
};