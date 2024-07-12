#pragma once

#include "pros/imu.hpp"
#include <cstdint>

struct AbstractGyro {
  /// @brief check if the sensor is ready
  /// @returns whether the sensor has finished calibrating
  virtual bool ready() = 0;

  /// @brief get the heading
  /// @returns the heading measurement
  virtual double heading() = 0;
};

struct AbstractImuGyro : public AbstractGyro, public pros::Imu {
  /// @brief create an object to forward function calls to an inertial sensor
  /// @param iport the port of the sensor
  /// @returns a abstract gyroscope object
  AbstractImuGyro(std::uint8_t iport) : pros::Imu(iport) { set_data_rate(5); }

  /// @brief check if the sensor is ready
  /// @returns whether the sensor has finished calibrating
  bool ready() override { return !is_calibrating(); }

  /// @brief get the heading
  /// @returns the heading measurement
  double heading() override {
    if (is_calibrating()) {
      return 0;
    } else {
      return get_heading();
    }
  }
};