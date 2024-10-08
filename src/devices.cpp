#include "devices.hpp"

#include "ports.h"
#include "subzerolib/api/chassis/tank-chassis.hpp"
#include "subzerolib/api/control/piston.hpp"
#include "subzerolib/api/odometry/gyro-odometry.hpp"
#include "subzerolib/api/odometry/kf-odometry.hpp"
#include "subzerolib/api/sensors/abstract-mean-gyro.hpp"
#include "subzerolib/api/util/logging.hpp"

#include <memory>

std::unique_ptr<pros::MotorGroup> mtr_l{
    new pros::MotorGroup({PORT_L1, PORT_L2, PORT_LT},
                         pros::MotorGears::blue,
                         pros::MotorUnits::deg)};
std::unique_ptr<pros::MotorGroup> mtr_r{
    new pros::MotorGroup({PORT_R1, PORT_R2, PORT_RT},
                         pros::MotorGears::blue,
                         pros::MotorUnits::deg)};
std::shared_ptr<TankChassis> chassis;

std::unique_ptr<pros::AbstractMotor> mtr_h_lift{
    new pros::Motor(PORT_LIFT, pros::MotorGears::green, pros::MotorUnits::deg)};
std::unique_ptr<pros::AbstractMotor> mtr_h_intake{new pros::Motor(
    PORT_INTAKE, pros::MotorGears::green, pros::MotorUnits::deg)};
std::unique_ptr<pros::AbstractMotor> mtr_wrist{
    new pros::Motor(PORT_WRIST, pros::MotorGears::red, pros::MotorUnits::deg)};

std::unique_ptr<pros::adi::DigitalOut> piston_clamp{
    new pros::adi::DigitalOut(ADI_CLAMP, false)};

Piston p_clamp({std::move(piston_clamp)});

/*
std::shared_ptr<AbstractGyro> imu_1{
    new AbstractImuGyro(IMU1, (1 * 360.0) / (1 * 360.0 + 0))};
// std::shared_ptr<AbstractGyro> imu_2{
//     new AbstractImuGyro(IMU2, (19 * 360.0) / (18 * 360.0 + 260))};
std::shared_ptr<AbstractGyro> mean_imu{
    new AbstractMeanGyro({imu_1})}; // TODO: reconf for 2 imus
std::shared_ptr<AbstractEncoder> enc_x{
    new AbstractRotationEncoder(PORT_X_ENC, true)};
std::shared_ptr<AbstractEncoder> enc_y{
    new AbstractRotationEncoder(PORT_Y_ENC, true)};
std::shared_ptr<Odometry> odom;
*/

void calibrate_imus() {
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
}

void configure_motors() {
  mtr_h_intake->set_brake_mode(pros::MotorBrake::brake);
  mtr_h_lift->set_brake_mode(pros::MotorBrake::hold);
  mtr_wrist->set_brake_mode(pros::MotorBrake::hold);
}

void configure_chassis() {
  chassis = TankChassis::Builder()
                .with_motor(TankChassis::motor_pos_e::left, std::move(mtr_l))
                .with_motor(TankChassis::motor_pos_e::right, std::move(mtr_r))
                .with_geometry(0.25) // TODO: FIXEME
                .with_rot_pref(0.3)
                .with_vel(1.76)
                .build();
}

void configure_odometry() {
  /*
  const double dt = 0.01;

  const double v_ah = std::pow(0.5, 2);
  const double v_al = std::pow(0.028, 2);

  // measurement variances
  const double vm_xh = std::pow(0.1, 2);
  const double vm_vh = std::pow(0.5, 2);

  const double vm_vl = std::pow(0.0025, 2);
  const double vm_al = std::pow(0.07, 2);

  Eigen::Vector<double, 8> initial_state{
      {0, 0, 0, 0, 0, 0, 0, 0}
  };
  Eigen::Matrix<double, 8, 8> initial_covariance;
  initial_covariance.setZero();
  initial_covariance.diagonal() = Eigen::Vector<double, 8>{
      {0.01, 0, 5, 0.01, 0, 5, 0.01, 0}
  };
  const double dt2 = 0.5 * dt * dt;
  Eigen::Matrix<double, 8, 8> state_transition_matrix{
      {1, dt, dt2, 0,  0,   0, 0,  0},
      {0,  1,  dt, 0,  0,   0, 0,  0},
      {0,  0,   1, 0,  0,   0, 0,  0},
      {0,  0,   0, 1, dt, dt2, 0,  0},
      {0,  0,   0, 0,  1,  dt, 0,  0},
      {0,  0,   0, 0,  0,   1, 0,  0},
      {0,  0,   0, 0,  0,   0, 1, dt},
      {0,  0,   0, 0,  0,   0, 0,  1},
  };
  Eigen::Matrix<double, 8, 8> observation_matrix{
      {0, 1, 0, 0, 0, 0, 0, 0},
      {0, 0, 1, 0, 0, 0, 0, 0},
      {0, 0, 1, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 1, 0, 0, 0},
      {0, 0, 0, 0, 0, 1, 0, 0},
      {0, 0, 0, 0, 0, 1, 0, 0},
      {0, 0, 0, 0, 0, 0, 1, 0},
      {0, 0, 0, 0, 0, 0, 0, 1},
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
  Eigen::Matrix<double, 8, 8> process_noise_covariance{
      {c4l, c3l,  c2l,   0,   0,    0,   0,   0},
      {c3l, c2l,  c1l,   0,   0,    0,   0,   0},
      {c2l, c1l, v_al,   0,   0,    0,   0,   0},
      {  0,   0,    0, c4l, c3l,  c2l,   0,   0},
      {  0,   0,    0, c3l, c2l,  c1l,   0,   0},
      {  0,   0,    0, c2l, c1l, v_al,   0,   0},
      {  0,   0,    0,   0,   0,    0, c4h, c3h},
      {  0,   0,    0,   0,   0,    0, c3h, c2h}
  };
  Eigen::Matrix<double, 8, 8> measurement_covariance;
  measurement_covariance.setZero();
  measurement_covariance.diagonal() = Eigen::Vector<double, 8>{
      {vm_vl, vm_al, vm_al, vm_vl, vm_al, vm_al, vm_xh, vm_vh}
  };

  KFOdometry::Builder builder(8, 0, 8);
  // TODO: FIXME
  builder.with_gyro(mean_imu)
      .with_x_enc(enc_x, {-0.045, 0.160 / 360.0})
      .with_y_enc(enc_y, {0.09, 0.160 / 360.0});
  builder.with_initial_state(initial_state)
      .with_initial_covariance(initial_covariance)
      .with_measurement_covariance(measurement_covariance)
      .with_state_transition_matrix(state_transition_matrix)
      //.with_control_matrix(control_matrix)
      .with_observation_matrix(observation_matrix)
      .with_process_noise_covariance(process_noise_covariance);
  odom = builder.build();
  odom->auto_update();
  */
}

void initialise_devices() {
  calibrate_imus();
  configure_motors();
  configure_chassis();
  configure_odometry();
}
