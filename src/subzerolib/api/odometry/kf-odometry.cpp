#include "subzerolib/api/odometry/kf-odometry.hpp"

void KFOdometry::set_heading(double heading) {
  GyroOdometry::set_heading(heading);
  auto state = KalmanFilter::get_state();
  auto cov = KalmanFilter::get_covariance();
  state(4) = heading;
  KalmanFilter::initialise(state, cov);
}

void KFOdometry::set_position(double x, double y) {
  GyroOdometry::set_position(x, y);
  auto state = KalmanFilter::get_state();
  auto cov = KalmanFilter::get_covariance();
  state(0) = x;
  state(2) = y;
  KalmanFilter::initialise(state, cov);
}

pose_s KFOdometry::get_pose() {
  Eigen::VectorXd state = KalmanFilter::get_state();
  return pose_s{state(0), state(2), state(4)};
}

pose_s KFOdometry::get_vel() {
  Eigen::VectorXd state = KalmanFilter::get_state();
  return pose_s{state(1), state(3), state(5)};
}

void KFOdometry::update() {
  GyroOdometry::update();
  if (is_enabled()) {
    auto pose = GyroOdometry::get_pose();
    auto vel = GyroOdometry::get_vel();
    Eigen::Vector<double, 4> meas{
        {vel.x, vel.y, pose.h, vel.h}
    };
    KalmanFilter::update(meas);
    auto raw_accel_a = pros::Imu::get_all_devices()[0].get_accel();
    auto raw_accel_b = pros::Imu::get_all_devices()[1].get_accel();
    point_s accel = rotate_acw(raw_accel_a.y, raw_accel_a.x, pose.h);
    accel = accel + rotate_acw(raw_accel_b.y, raw_accel_b.x, pose.h);
    accel = 9.8 * accel / 2.0;
    if (std::isnan(accel.x) || std::isnan(accel.y)) {
      accel.x = 0;
      accel.y = 0;
    }
    KalmanFilter::predict(Eigen::Vector<double, 2>{accel.x, accel.y});
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

  if (filter->nu == 0) {
    filter->predict();
  } else {
    Eigen::VectorXd vec;
    vec.resize(filter->nu);
    vec.setZero();
    filter->predict(vec);
  }

  return std::shared_ptr<KFOdometry>{
      new KFOdometry{gyro_odom, filter}
  };
}
