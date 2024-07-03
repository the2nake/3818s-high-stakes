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

void balance_vels(std::vector<control_components_s> &vels, double max_v = 1.0,
                  double rot_pref = 0.5);

template <typename T>
void balance_mapped_vels(std::map<T, control_components_s> &vel_map,
                         double max_v, double rot_pref) {
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