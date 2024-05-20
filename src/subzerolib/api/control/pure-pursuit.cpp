#include "subzerolib/api/control/pure-pursuit.hpp"
#include <memory>

PurePursuitController::PurePursuitController(std::shared_ptr<Chassis> ichassis)
    : chassis(std::move(ichassis)) {}