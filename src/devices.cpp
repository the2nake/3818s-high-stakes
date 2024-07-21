#include "devices.hpp"

#include "ports.h"
#include "subzerolib/api/chassis/star-chassis.hpp"
#include "subzerolib/api/odometry/gyro-odometry.hpp"
#include "subzerolib/api/odometry/kf-odometry.hpp"
#include "subzerolib/api/sensors/abstract-mean-gyro.hpp"
#include "subzerolib/api/util/logging.hpp"

#include "pros/motors.hpp"
#include <memory>

std::unique_ptr<pros::Motor> fl{new pros::Motor(
    -DRIVE_FL_PORT, pros::v5::MotorGears::green, pros::v5::MotorUnits::deg)};
std::unique_ptr<pros::Motor> fr{new pros::Motor(
    DRIVE_FR_PORT, pros::v5::MotorGears::green, pros::v5::MotorUnits::deg)};
std::unique_ptr<pros::Motor> ml{new pros::Motor(
    -DRIVE_ML_PORT, pros::v5::MotorGears::green, pros::v5::MotorUnits::deg)};
std::unique_ptr<pros::Motor> mr{new pros::Motor(
    DRIVE_MR_PORT, pros::v5::MotorGears::green, pros::v5::MotorUnits::deg)};
std::unique_ptr<pros::Motor> br{new pros::Motor(
    DRIVE_BR_PORT, pros::v5::MotorGears::green, pros::v5::MotorUnits::deg)};
std::unique_ptr<pros::Motor> bl{new pros::Motor(
    -DRIVE_BL_PORT, pros::v5::MotorGears::green, pros::v5::MotorUnits::deg)};
std::shared_ptr<Chassis> chassis;

std::shared_ptr<AbstractGyro> imu1{
    new AbstractImuGyro(IMU1_PORT, (18 * 360.0) / (17 * 360.0 + 283))};
std::shared_ptr<AbstractGyro> imu2{
    new AbstractImuGyro(IMU2_PORT, (19 * 360.0) / (18 * 360.0 + 260))};
std::shared_ptr<AbstractGyro> mean_imu{new AbstractMeanGyro({imu1, imu2})};
std::shared_ptr<AbstractEncoder> odom_x{
    new AbstractRotationEncoder(PORT_X_ENC, true)};
std::shared_ptr<AbstractEncoder> odom_y{
    new AbstractRotationEncoder(PORT_Y_ENC, true)};
std::shared_ptr<Odometry> odom;

void initialise_devices() {
  auto imus = pros::Imu::get_all_devices();
  for (auto device : imus) {
    subzero::log("[i]: resetting imu on port %d", device.get_port());
    device.reset();
  }
  for (auto device : imus) {
    while (device.is_calibrating())
      pros::delay(100);
    subzero::log("[i]: imu on port %d ready", device.get_port());
  }

  pros::delay(500);

  chassis =
      StarChassis::Builder()
          .with_motors(StarChassis::motor_pos_e::front_left, std::move(fl))
          .with_motors(StarChassis::motor_pos_e::front_right, std::move(fr))
          .with_motors(StarChassis::motor_pos_e::boost_left, std::move(ml))
          .with_motors(StarChassis::motor_pos_e::boost_right, std::move(mr))
          .with_motors(StarChassis::motor_pos_e::back_left, std::move(bl))
          .with_motors(StarChassis::motor_pos_e::back_right, std::move(br))
          .with_geometry(0.35, 0.37)
          .with_rot_pref(0.3)
          .build();

  const double dt = 0.01;

  // TODO: tune process noise
  const double v_ah = std::pow(10, 2);
  // high for unpredictable acceleration? control input seems better
  const double v_al = std::pow(0.5, 2);

  // measurement variances
  const double vm_xh = std::pow(0.1, 2);
  const double vm_vh = std::pow(0.5, 2);
  const double vm_xl = std::pow(0.01, 2);
  const double vm_vl = std::pow(0.1, 2);

  Eigen::Vector<double, 9> initial_state{
      {0, 0, 0, 0, 0, 0, 0, 0, 0}
  };
  Eigen::Matrix<double, 9, 9> initial_covariance;
  initial_covariance.setZero();
  initial_covariance.diagonal() = Eigen::Vector<double, 9>{
      {5, 5, 5, 5, 5, 5, 200, 200, 200}
  };
  const double dt2 = dt * dt * 0.5;
  Eigen::Matrix<double, 9, 9> state_transition_matrix{
      {1, dt, dt2, 0,  0,   0, 0,  0,   0},
      {0,  1,  dt, 0,  0,   0, 0,  0,   0},
      {0,  0,   1, 0,  0,   0, 0,  0,   0},
      {0,  0,   0, 1, dt, dt2, 0,  0,   0},
      {0,  0,   0, 0,  1,  dt, 0,  0,   0},
      {0,  0,   0, 0,  0,   1, 0,  0,   0},
      {0,  0,   0, 0,  0,   0, 1, dt, dt2},
      {0,  0,   0, 0,  0,   0, 0,  1,  dt},
      {0,  0,   0, 0,  0,   0, 0,  0,   1}
  };
  Eigen::Matrix<double, 6, 9> observation_matrix{
      {1, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 1, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 1, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 1, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 1, 0},
  };
  const double c4 = 0.25 * dt * dt * dt * dt;
  const double c3 = 0.5 * dt * dt * dt;
  const double c2 = dt * dt;
  const double c4h = v_ah * c4;
  const double c3h = v_ah * c3;
  const double c2h = v_ah * c2;
  const double c1h = v_ah * dt;
  const double c4l = v_al * c4;
  const double c3l = v_al * c3;
  const double c2l = v_al * c2;
  const double c1l = v_al * dt;
  Eigen::Matrix<double, 9, 9> process_noise_covariance{
      {c4l, c3l, c2l,   0,   0,   0,   0,   0,   0},
      {c3l, c2l, c1l,   0,   0,   0,   0,   0,   0},
      {c2l, c1l,   1,   0,   0,   0,   0,   0,   0},
      {  0,   0,   0, c4l, c3l, c2l,   0,   0,   0},
      {  0,   0,   0, c3l, c2l, c1l,   0,   0,   0},
      {  0,   0,   0, c2l, c1l,   1,   0,   0,   0},
      {  0,   0,   0,   0,   0,   0, c4h, c3h, c2h},
      {  0,   0,   0,   0,   0,   0, c3h, c2h, c1h},
      {  0,   0,   0,   0,   0,   0, c2h, c1h,   1}
  };
  Eigen::Matrix<double, 6, 6> measurement_covariance;
  measurement_covariance.setZero();
  measurement_covariance.diagonal() = Eigen::Vector<double, 6>{
      {vm_xl, vm_vl, vm_xl, vm_vl, vm_xh, vm_vh}
  };

  // TODO: tune filtering parameters properly

  KFOdometry::Builder builder(9, 0, 6);
  builder.with_gyro(mean_imu)
      .with_x_enc(odom_x, {-0.045, 0.160 / 360.0})
      .with_y_enc(odom_y, {0.09, 0.160 / 360.0});
  builder.with_initial_state(initial_state)
      .with_initial_covariance(initial_covariance)
      .with_measurement_covariance(measurement_covariance)
      .with_state_transition_matrix(state_transition_matrix)
      .with_observation_matrix(observation_matrix)
      .with_process_noise_covariance(process_noise_covariance);
  odom = builder.build();
  odom->auto_update();
}