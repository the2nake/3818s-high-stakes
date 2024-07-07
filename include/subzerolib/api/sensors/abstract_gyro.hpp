#pragma once

#include "pros/imu.hpp"
#include <cstdint>

struct AbstractGyro {
  virtual double heading() = 0;
  virtual bool ready() = 0;
};

// class to forward function calls
struct AbstractImuGyro : public AbstractGyro, public pros::Imu {
  AbstractImuGyro(std::uint8_t iport) : pros::Imu(iport) {
    set_data_rate(5);
  }

  bool ready() override { return !is_calibrating(); }

  double heading() override {
    if (is_calibrating()) {
      return 0;
    } else {
      return get_heading();
    }
  }
};