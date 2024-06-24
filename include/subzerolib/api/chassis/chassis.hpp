#pragma once

class Chassis {
public:
  /// @brief move the robot in relative reference frame
  ///
  /// @param x lateral component
  /// @param y vertical component
  /// @param r rotational component
  virtual void move(double x, double y, double r) = 0;

  /// @brief specify a preference for combining linear and angular components
  /// @param rot_pref a value in the range [0.0, 1.0]
  virtual void set_rot_pref(double irot_pref = 0.5) = 0;

protected:
  Chassis() {}
};