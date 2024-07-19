#include "devices.hpp"

#include "ports.h"
#include "subzerolib/api/chassis/star-chassis.hpp"
#include "subzerolib/api/odometry/gyro-odometry.hpp"
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

  odom = GyroOdometry::Builder()
             .with_gyro(mean_imu)
             .with_x_enc(odom_x, {-0.045, 0.160 / 360.0})
             .with_y_enc(odom_y, {0.09, 0.160 / 360.0})
             .build();

  odom->auto_update();
}