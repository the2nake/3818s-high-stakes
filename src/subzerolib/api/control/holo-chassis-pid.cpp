#include "subzerolib/api/control/holo-chassis-pid.hpp"

// linv is ignored
void HoloChassisPID::approach_pose(pose_s target, double linv) {
  pose_s dev = target - odom->get_pose(); // closest angle calc already done
  x_pid->update(dev.x);
  y_pid->update(dev.y);
  r_pid->update(dev.h);
  chassis->move(x_pid->get_output(), y_pid->get_output(), r_pid->get_output());
}

HoloChassisPID::Builder &
HoloChassisPID::Builder::with_chassis(
    std::shared_ptr<Chassis> ichassis) {
  if (ichassis != nullptr) {
    bchassis = std::move(ichassis);
  }
  return *this;
}

HoloChassisPID::Builder &
HoloChassisPID::Builder::with_odom(
    std::shared_ptr<Odometry> iodom) {
  if (iodom != nullptr) {
    bodom = std::move(iodom);
  }
  return *this;
}

HoloChassisPID::Builder &
HoloChassisPID::Builder::with_pid(
    HoloChassisPID::pid_dimension_e dimension, double kp, double ki,
    double kd) {
  PIDF *pid = new PIDF(kp, ki, kd, 0.0);
  // if the dimension has been set before, it *should* disappear and memory
  // deallocated easily
  switch (dimension) {
  case HoloChassisPID::pid_dimension_e::x:
    bx_pid = std::unique_ptr<PIDF>(pid);
    break;
  case HoloChassisPID::pid_dimension_e::y:
    by_pid = std::unique_ptr<PIDF>(pid);
    break;
  case HoloChassisPID::pid_dimension_e::r:
    br_pid = std::unique_ptr<PIDF>(pid);
    break;
  }
  return *this;
}

std::shared_ptr<HoloChassisPID> HoloChassisPID::Builder::build() {
  HoloChassisPID *controller = new HoloChassisPID();
  if (bchassis == nullptr || bodom == nullptr || bx_pid == nullptr ||
      by_pid == nullptr || br_pid == nullptr) {
    return nullptr;
  }

  controller->chassis = bchassis;
  controller->odom = bodom;
  controller->x_pid = std::move(bx_pid);
  controller->y_pid = std::move(by_pid);
  controller->r_pid = std::move(br_pid);

  return std::shared_ptr<HoloChassisPID>(controller);
}