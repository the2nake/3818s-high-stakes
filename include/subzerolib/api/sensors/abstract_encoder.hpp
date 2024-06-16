#pragma once

#include "pros/adi.hpp"
#include "pros/rotation.hpp"

struct AbstractEncoder {
  virtual double get_deg() = 0;
};

// forward function calls
struct AbstractRotationEncoder : public AbstractEncoder, public pros::Rotation {
  double get_deg() override { return pros::Rotation::get_position() / 100.0; }
};

// forward function calls
struct AbstractADIEncoder : public AbstractEncoder, public pros::adi::Encoder {
  double get_deg() override { return pros::adi::Encoder::get_value(); }
};