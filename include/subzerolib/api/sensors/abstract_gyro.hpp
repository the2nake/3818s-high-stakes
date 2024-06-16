#pragma once

#include "pros/imu.hpp"
#include <cstdint>

class AbstractGyro {
public:
  virtual double get_heading() = 0;
};

// class to forward function calls
struct AbstactImuGyro : public AbstractGyro, public pros::Imu {
  AbstactImuGyro(std::uint8_t iport) : Imu(iport) {
    pros::Imu::set_data_rate(5);
  }

  double get_heading() override { return pros::Imu::get_heading(); }
};