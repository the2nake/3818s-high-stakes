#pragma once

#include "subzerolib/api/sensors/abstract-gyro.hpp"
#include "subzerolib/api/util/math.hpp"
#include <initializer_list>

class AbstractMeanGyro : public AbstractGyro {
public:
  AbstractMeanGyro(std::shared_ptr<AbstractGyro> i_gyro1,
                   std::shared_ptr<AbstractGyro> i_gyro2) {
    gyros.push_back(std::move(i_gyro1));
    gyros.push_back(std::move(i_gyro2));
  }
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
  virtual double degrees() {
    if (gyros.size() < 1) {
      return 0.0;
    }
    const double index = gyros[0]->degrees();
    double result = index;
    for (auto &gyro : gyros) {
      result += shorter_turn(index, gyro->degrees(), 360.0) / gyros.size();
    }
    return result;
  }

private:
  std::vector<std::shared_ptr<AbstractGyro>> gyros;
};
