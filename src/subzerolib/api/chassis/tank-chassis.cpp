#include "subzerolib/api/chassis/tank-chassis.hpp"
#include "subzerolib/api/util/helper.hpp"

void TankChassis::move(double x, double y, double r) {
  // do not clamp velocities, keep original ratios
  insert_or_modify(vels, TankChassis::motor_pos_e::left, {y, r});
  insert_or_modify(vels, TankChassis::motor_pos_e::right, {y, -r});
  balance_mapped_vels(vels, 1.0, rot_pref);
  move_with_map();
}

void TankChassis::move_with_map() {
  for (auto &pair : vels) {
    position_ptr_map.at(pair.first)->move_voltage(12000 * pair.second.sum());
  }
}
