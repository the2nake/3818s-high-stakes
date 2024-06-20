#pragma once

#include "pros/imu.hpp"
#include <cstdint>

struct AbstractGyro {
  virtual double get_heading() = 0;
};

// class to forward function calls
struct AbstractImuGyro : public AbstractGyro, public pros::Imu {
  AbstractImuGyro(std::uint8_t iport) : pros::Imu(iport) {
    pros::Imu::set_data_rate(5);
  }

  double get_heading() override { return pros::Imu::get_heading(); }
};