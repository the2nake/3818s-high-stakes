#pragma once

#include "pros/rtos.hpp"
#include "subzerolib/api/odometry/odometry.hpp"
#include "subzerolib/api/sensors/abstract_encoder.hpp"
#include "subzerolib/api/sensors/abstract_gyro.hpp"

#include <memory>

class ImuOdometry : public Odometry {
public:
  void set_heading(double h) override;
  void set_position(double x, double y) override;

  pose_s get_pose() override;
  point_s get_vel() override;

  void update() override;

private:
  pros::Mutex state_mutex;

  AbstractGyro *gyro;
  AbstractEncoder *x_enc;
  AbstractEncoder *y_enc;
  encoder_conf_s x_conf;
  encoder_conf_s y_conf;

  uint32_t prev_timestamp = 0;
  double prev_heading = 0.0;
  double prev_x_enc_val = 0.0;
  double prev_y_enc_val = 0.0;

  pose_s pose;

  ImuOdometry() {}
  void lock() {
    while (!this->state_mutex.take(5)) {
      pros::delay(1);
    }
  }

public:
  class ImuOdometryBuilder {
  public:
    ImuOdometryBuilder &with_gyro(AbstractGyro *igyro);
    ImuOdometryBuilder &with_x_enc(AbstractEncoder *encoder,
                                   encoder_conf_s conf);
    ImuOdometryBuilder &with_y_enc(AbstractEncoder *encoder,
                                   encoder_conf_s conf);

    std::shared_ptr<ImuOdometry> build();

  private:
    AbstractGyro *gyro;
    AbstractEncoder *x_enc;
    AbstractEncoder *y_enc;
    encoder_conf_s x_conf;
    encoder_conf_s y_conf;
  };
};