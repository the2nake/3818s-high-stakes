#pragma once

#include "pros/rotation.hpp"

class AbstractEncoder {
public:
  virtual double get_deg() = 0;
};

// class to forward function calls
class AbstactRotationEncoder : public AbstractEncoder, public pros::Rotation {
public:
  double get_deg() override { return pros::Rotation::get_position() / 100.0; }
};