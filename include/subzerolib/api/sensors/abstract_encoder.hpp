#pragma once

#include "pros/adi.hpp"
#include "pros/rotation.hpp"

struct AbstractEncoder {
  virtual double get_deg() = 0;
  virtual void set_deg(double deg) = 0;
};

// forward function calls
struct AbstractRotationEncoder : public AbstractEncoder, public pros::Rotation {
  AbstractRotationEncoder(std::uint8_t iport, bool rev)
      : pros::Rotation(iport) {
    pros::Rotation::set_data_rate(5);
    pros::Rotation::set_reversed(rev);
  }
  double get_deg() override { return pros::Rotation::get_position() / 100.0; }
  void set_deg(double deg) override { pros::Rotation::set_position(100 * deg); }
};

// forward function calls
class AbstractADIEncoder : public AbstractEncoder, public pros::adi::Encoder {
public:
  AbstractADIEncoder(std::uint8_t adi_port_top, std::uint8_t adi_port_bottom,
                     bool reversed = false)
      : pros::adi::Encoder(adi_port_top, adi_port_bottom, reversed) {}
  AbstractADIEncoder(pros::adi::ext_adi_port_tuple_t tuple,
                     bool reversed = false)
      : pros::adi::Encoder(tuple, reversed) {}
  double get_deg() override { return pros::adi::Encoder::get_value() + offset; }
  void set_deg(double deg) override {
    pros::adi::Encoder::reset();
    offset = deg;
  }

private:
  double offset = 0.0;
};