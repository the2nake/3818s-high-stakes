#pragma once

#include "subzerolib/api/chassis/chassis.hpp"
#include <memory>

class PurePursuitController {
public:
  PurePursuitController(std::shared_ptr<Chassis> ichassis);

private:
  std::shared_ptr<Chassis> chassis;

};