#pragma once
#include <map>
#include <vector>

struct control_components_s {
  control_components_s(double ilin = 0, double irot = 0)
      : lin(ilin), rot(irot) {}
  double lin;
  double rot;

  double sum();
  double sum(double rot_pref);
};

/// @brief balances control velocities
/// @param vels reference to the std::vector of control components
/// @param max_v the maximum velocity
/// @param rot_pref the preference for rotation over linear velocity
void balance_vels(std::vector<control_components_s> &vels,
                  double max_v = 1.0,
                  double rot_pref = 0.5);

/// @brief balanced control velocities inside a map
/// @tparam T anything mapped to a control component
/// @param vel_map reference to the std::map
/// @param max_vel the maximum velocity
/// @param rot_pref the preference for rotation over linear velocity
template <typename T>
void balance_mapped_vels(std::map<T, control_components_s> &vel_map,
                         double max_v,
                         double rot_pref) {
  std::vector<T> keys;
  std::vector<control_components_s> components;
  for (auto &pair : vel_map) {
    keys.push_back(pair.first);
    components.push_back(pair.second);
  }
  balance_vels(components, max_v, rot_pref);
  for (auto &key : keys) {
    auto i = &key - &keys[0];
    vel_map.at(key) = components[i];
  }
}