#include "subzerolib/api/odometry/imu_odometry.hpp"

ImuOdometry::ImuOdometryBuilder &
ImuOdometry::ImuOdometryBuilder::with_gyro(AbstractGyro *igyro) {
  gyro = igyro;
  return *this;
}
ImuOdometry::ImuOdometryBuilder &
ImuOdometry::ImuOdometryBuilder::with_x_enc(AbstractEncoder *encoder,
                                            encoder_conf_s conf) {
  x_enc = encoder;
  x_conf = conf;
  return *this;
}
ImuOdometry::ImuOdometryBuilder &
ImuOdometry::ImuOdometryBuilder::with_y_enc(AbstractEncoder *encoder,
                                            encoder_conf_s conf) {
  y_enc = encoder;
  y_conf = conf;
  return *this;
}

std::shared_ptr<ImuOdometry> ImuOdometry::ImuOdometryBuilder::build() {
  if (gyro == nullptr) {
    return nullptr;
  }
  if (x_enc == nullptr) {
    return nullptr;
  }
  if (y_enc == nullptr) {
    return nullptr;
  }

  std::shared_ptr<ImuOdometry> odom = std::make_shared<ImuOdometry>();
  odom->prev_timestamp = pros::millis();
  odom->gyro = gyro;
  odom->x_enc = x_enc;
  odom->x_conf = x_conf;
  odom->y_enc = y_enc;
  odom->y_conf = y_conf;
  odom->pose = pose_s{0, 0, 0};

  return odom;
}