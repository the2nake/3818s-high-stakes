#pragma once

#include "subzerolib/api/chassis/chassis.hpp"
#include "subzerolib/api/util/controls.hpp"

#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"
#include <map>
#include <memory>

class StarChassis : public Chassis {
public:
  enum class motor_position_e {
    front_left,
    front_right,
    boost_left,
    boost_right,
    back_left,
    back_right
  };

  /// @brief move in relative reference frame
  ///
  /// @param x lateral component
  /// @param y vertical component
  /// @param r rotational component
  void move(double x, double y, double r) override;

  /// @brief specify a preference for combining linear and angular components
  /// @param rot_pref a value in the range [0.0, 1.0]
  void set_rot_pref(double irot_pref = 0.5) override;

private:
  StarChassis() {}
  void move_with_map();

  double boost_radius;
  double corner_radius;
  double rot_pref;

  std::map<StarChassis::motor_position_e, control_components_s> vels = {};
  std::map<StarChassis::motor_position_e,
           std::unique_ptr<pros::AbstractMotor> &>
      position_ptr_map = {};
  std::unique_ptr<pros::AbstractMotor> front_left = nullptr;
  std::unique_ptr<pros::AbstractMotor> front_right = nullptr;
  std::unique_ptr<pros::AbstractMotor> back_left = nullptr;
  std::unique_ptr<pros::AbstractMotor> back_right = nullptr;
  std::unique_ptr<pros::AbstractMotor> boost_left = nullptr;
  std::unique_ptr<pros::AbstractMotor> boost_right = nullptr;

public:
  class Builder {
  public:
    /// @brief specify a motor for the chassis
    ///
    /// the chassis motor should be reversed such that positive voltage moves
    /// the drive forward
    ///
    /// @param position the motor position
    /// @param motor a unique pointer to the motor
    /// @returns a reference to the builder object
    Builder &with_motors(motor_position_e position,
                         std::unique_ptr<pros::Motor> motor);

    /// @brief specify a motor for the chassis
    ///
    /// the chassis motor should be reversed such that positive voltage moves
    /// the drive forward
    ///
    /// @param position the motor position
    /// @param motor a unique pointer to the motor
    /// @returns a reference to the builder object
    Builder &with_motors(motor_position_e position,
                         std::unique_ptr<pros::AbstractMotor> motor);

    /// @brief specify star drive geometry
    /// @param iboost_radius the radius from the centre to the middle of the
    /// boost wheels
    /// @param icorner_radius the radius from the centre to the middle of the
    /// corner wheels
    /// @returns a reference to the builder object
    Builder &with_geometry(double iboost_radius, double icorner_radius);

    /// @brief specify a preference for combining linear and angular components
    /// @param irot_pref a value in the range [0.0, 1.0]
    /// @returns a reference to the builder object
    Builder &with_rot_pref(double irot_pref = 0.5);

    /// @brief creates the star drive chassis object
    /// @returns a shared pointer to the created object
    std::shared_ptr<StarChassis> build();

  private:
    bool try_copy(std::unique_ptr<pros::AbstractMotor> &target,
                  std::unique_ptr<pros::AbstractMotor> &origin);
    double bboost_radius;
    double bcorner_radius;
    double brot_pref;
    std::unique_ptr<pros::AbstractMotor> bfront_left = nullptr;
    std::unique_ptr<pros::AbstractMotor> bfront_right = nullptr;
    std::unique_ptr<pros::AbstractMotor> bback_left = nullptr;
    std::unique_ptr<pros::AbstractMotor> bback_right = nullptr;
    std::unique_ptr<pros::AbstractMotor> bboost_left = nullptr;
    std::unique_ptr<pros::AbstractMotor> bboost_right = nullptr;
  };
};