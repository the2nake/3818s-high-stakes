#pragma once

class LinearMotionProfile {
public:
  struct profile_point_s {
    profile_point_s(double it, double ix, double iv) : t(it), x(ix), v(iv) {}

    double t;
    double x;
    double v;
  };

  /// @brief sets the steps between generated points
  /// @param step_length distance covered by each step
  virtual void set_resolution(double step_length) = 0;

  /// @brief generate the motion profile for a given distance
  /// @param distance final distance covered
  virtual void generate(double distance) = 0;

  /// @brief queries a point by time
  /// @param time the time corresponding to the point
  virtual profile_point_s get_point_at_time(double time) = 0;

  /// @brief queries a point by distance
  /// @param distance the distance corresponding to the point
  virtual profile_point_s get_point_at_distance(double distance) = 0;

  virtual double get_max_vel() = 0;
  virtual double get_max_accel() = 0;
  virtual double get_max_decel() = 0;

protected:
  LinearMotionProfile() {}
};
