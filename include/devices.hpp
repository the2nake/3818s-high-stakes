#pragma once

#include "subzerolib/api/chassis/chassis.hpp"
#include "subzerolib/api/odometry/odometry.hpp"
#include "subzerolib/api/sensors/abstract-encoder.hpp"
#include "subzerolib/api/sensors/abstract-gyro.hpp"

#include "pros/motors.hpp"
#include <memory>

extern std::unique_ptr<pros::Motor> fl;
extern std::unique_ptr<pros::Motor> fr;
extern std::unique_ptr<pros::Motor> ml;
extern std::unique_ptr<pros::Motor> mr;
extern std::unique_ptr<pros::Motor> br;
extern std::unique_ptr<pros::Motor> bl;
extern std::shared_ptr<Chassis> chassis;

extern std::shared_ptr<AbstractGyro> imu1;
extern std::shared_ptr<AbstractGyro> imu2;
extern std::shared_ptr<AbstractGyro> mean_imu;
extern std::shared_ptr<AbstractEncoder> odom_x;
extern std::shared_ptr<AbstractEncoder> odom_y;
extern std::shared_ptr<Odometry> odom;

void initialise_devices();
