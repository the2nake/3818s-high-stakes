#include "subzerolib/api/control/pid.hpp"
#include "pros/rtos.hpp"

void PIDF::reset() {
  last_update = pros::millis();
  prev_err = std::nan("");
  total_err = 0.0;
  output = 0.0;
}

void PIDF::update(double error) {
  uint32_t now = pros::millis();
  double dt = now - last_update;
  if (std::isnan(error)) {
    return;
  }
  total_err += error * dt;
  double p = kp * error;
  double i = ki * total_err;
  double d = 0.0;
  if ((!std::isnan(prev_err)) && (dt != 0.0)) {
    d = kd * (error - prev_err) / dt;
  }
  output = ff + p + i + d;
  prev_err = error;
  last_update = now;
}