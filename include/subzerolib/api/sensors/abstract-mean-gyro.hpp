#pragma once

#include "subzerolib/api/sensors/abstract-gyro.hpp"
#include "subzerolib/api/util/math.hpp"
#include <initializer_list>
#include <numeric>

class AbstractMeanGyro : public AbstractGyro {
public:
  AbstractMeanGyro(
      std::initializer_list<std::shared_ptr<AbstractGyro>> i_gyros) {
    for (auto &gyro : i_gyros) {
      gyros.push_back(std::move(gyro));
    }
  }

  /// @brief check if the sensor is ready
  /// @returns whether the sensor has finished calibrating
  virtual bool ready() {
    for (auto &gyro : gyros) {
      if (!gyro->ready()) {
        return false;
      }
    }
    return true;
  }

  /// @brief get the heading
  /// @returns the heading measurement
  virtual double heading() {
    if (gyros.size() < 1) {
      return 0.0;
    }
    const double index = gyros[0]->heading();
    double result = index;
    for (auto &gyro : gyros) {
      result += shorter_turn(index, gyro->heading()) / gyros.size();
    }
    return mod(result, 360.0);
  }

private:
  std::vector<std::shared_ptr<AbstractGyro>> gyros;
};