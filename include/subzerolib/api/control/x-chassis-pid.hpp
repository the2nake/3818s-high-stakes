#pragma once

#include "subzerolib/api/chassis/x-chassis.hpp"
#include "subzerolib/api/control/chassis-controller.hpp"
#include "subzerolib/api/control/pid.hpp"
#include "subzerolib/api/odometry/odometry.hpp"
#include <memory>

class XChassisPID : public ChassisController {
public:
  enum class pid_dimension_e {
    x,
    y,
    r,
  };

  void approach_pose(pose_s target, double linv = std::nan("")) override;
  void brake() override { chassis->move(0, 0, 0); }

private:
  std::shared_ptr<XChassis> chassis = nullptr;
  std::shared_ptr<Odometry> odom = nullptr;
  std::unique_ptr<PIDF> x_pid = nullptr;
  std::unique_ptr<PIDF> y_pid = nullptr;
  std::unique_ptr<PIDF> r_pid = nullptr;

  XChassisPID() {}

public:
  class XChassisPIDBuilder {
  public:
    // TODO: move builder impl
    XChassisPIDBuilder &with_chassis(std::shared_ptr<XChassis> ichassis) {
      if (ichassis != nullptr) {
        bchassis = std::move(ichassis);
      }
      return *this;
    }

    XChassisPIDBuilder &with_odom(std::shared_ptr<Odometry> iodom) {
      if (iodom != nullptr) {
        bodom = std::move(iodom);
      }
      return *this;
    }

    XChassisPIDBuilder &with_pid(pid_dimension_e dimension, double kp,
                                 double ki, double kd) {
      PIDF *pid = new PIDF(kp, ki, kd, 0.0);
      // if the dimension has been set before, it *should* disappear and memory
      // deallocated easily
      switch (dimension) {
      case XChassisPID::pid_dimension_e::x:
        bx_pid = std::unique_ptr<PIDF>(pid);
        break;
      case XChassisPID::pid_dimension_e::y:
        by_pid = std::unique_ptr<PIDF>(pid);
        break;
      case XChassisPID::pid_dimension_e::r:
        br_pid = std::unique_ptr<PIDF>(pid);
        break;
      }
      return *this;
    }

    std::shared_ptr<XChassisPID> build() {
      XChassisPID *controller = new XChassisPID();
      if (bchassis == nullptr || bodom == nullptr || bx_pid == nullptr ||
          by_pid == nullptr || br_pid == nullptr) {
        return nullptr;
      }

      controller->chassis = bchassis;
      controller->odom = bodom;
      controller->x_pid = std::move(bx_pid);
      controller->y_pid = std::move(by_pid);
      controller->r_pid = std::move(br_pid);

      return std::shared_ptr<XChassisPID>(controller);
    }

  private:
    std::shared_ptr<XChassis> bchassis = nullptr;
    std::shared_ptr<Odometry> bodom = nullptr;
    std::unique_ptr<PIDF> bx_pid = nullptr;
    std::unique_ptr<PIDF> by_pid = nullptr;
    std::unique_ptr<PIDF> br_pid = nullptr;
  };
};