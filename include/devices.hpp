#pragma once

#include "subzerolib/api/chassis/chassis.hpp"
#include "subzerolib/api/odometry/odometry.hpp"
#include "subzerolib/api/sensors/abstract-encoder.hpp"
#include "subzerolib/api/sensors/abstract-gyro.hpp"

#include "pros/motor_group.hpp"
#include <memory>

extern std::unique_ptr<pros::MotorGroup> mtr_l;
extern std::unique_ptr<pros::MotorGroup> mtr_r;
extern std::shared_ptr<Chassis> chassis;

extern std::shared_ptr<AbstractGyro> imu_1;
extern std::shared_ptr<AbstractGyro> imu_2;
extern std::shared_ptr<AbstractGyro> mean_imu;

extern std::shared_ptr<AbstractEncoder> enc_x;
extern std::shared_ptr<AbstractEncoder> enc_y;
extern std::shared_ptr<Odometry> odom;

void initialise_devices();
