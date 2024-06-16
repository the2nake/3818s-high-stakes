#pragma once

#include "pros/adi.hpp"
#include "pros/rotation.hpp"

struct abstract_encoder_s {
  virtual double get_deg() = 0;
};

// forward function calls
struct abstract_rotation_encoder_s : public abstract_encoder_s, public pros::Rotation {
  double get_deg() override { return pros::Rotation::get_position() / 100.0; }
};

// forward function calls
struct abstract_adi_encoder_s : public abstract_encoder_s, public pros::adi::Encoder {
  double get_deg() override { return pros::adi::Encoder::get_value(); }
};