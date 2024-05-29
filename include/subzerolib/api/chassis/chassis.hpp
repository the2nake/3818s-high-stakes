#pragma once

class Chassis {
public:
  /// @brief move the robot in relative reference frame
  ///
  /// @param x lateral component
  /// @param y vertical component
  /// @param r rotational component
  virtual void move(double x, double y, double r) = 0;

protected:
  Chassis() {}
};