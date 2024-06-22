#include "subzerolib/api/control/x-chassis-pid.hpp"

// linv is ignored
void XChassisPID::approach_pose(pose_s target, double linv) {
  pose_s dev = target - odom->get_pose(); // closest angle calc already done
  x_pid->update(dev.x);
  y_pid->update(dev.y);
  r_pid->update(dev.h);
  chassis->move(x_pid->get_output(), y_pid->get_output(), r_pid->get_output());
}