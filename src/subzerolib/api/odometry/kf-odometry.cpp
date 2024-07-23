#include "subzerolib/api/odometry/kf-odometry.hpp"

void KFOdometry::set_heading(double heading) {
  GyroOdometry::set_heading(heading);
  auto state = KalmanFilter::get_state();
  auto cov = KalmanFilter::get_covariance();
  state(6) = heading;
  KalmanFilter::initialise(state, cov);
}

void KFOdometry::set_position(double x, double y) {
  GyroOdometry::set_position(x, y);
  auto state = KalmanFilter::get_state();
  auto cov = KalmanFilter::get_covariance();
  state(0) = x;
  state(3) = y;
  KalmanFilter::initialise(state, cov);
}

pose_s KFOdometry::get_pose() {
  Eigen::VectorXd state = KalmanFilter::get_state();
  return pose_s{state(0), state(3), state(6)};
}

pose_s KFOdometry::get_vel() {
  Eigen::VectorXd state = KalmanFilter::get_state();
  return pose_s{state(1), state(4), state(7)};
}

/// @brief trigger an update tick
void KFOdometry::update() {
  GyroOdometry::update();
  if (is_enabled()) {
    KalmanFilter::predict();
    auto pose = GyroOdometry::get_pose();
    auto vel = GyroOdometry::get_vel();
    Eigen::Vector<double, 6> meas{pose.x, vel.x, pose.y, vel.y, pose.h, vel.h};
    KalmanFilter::update(meas);
  }
}

void KFOdometry::set_enabled(bool v) { GyroOdometry::set_enabled(v); }

bool KFOdometry::is_enabled() { return GyroOdometry::is_enabled(); }

std::shared_ptr<KFOdometry> KFOdometry::Builder::build() {
  auto gyro_odom = GyroOdometry::Builder::build();
  auto filter = KalmanFilter::Builder::build();

  if (gyro_odom == nullptr || filter == nullptr) {
    return nullptr;
  }

  return std::shared_ptr<KFOdometry>{
      new KFOdometry{gyro_odom, filter}
  };
}