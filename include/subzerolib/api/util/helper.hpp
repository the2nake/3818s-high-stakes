#pragma once

#include "subzerolib/api/geometry/circle.hpp"
#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/geometry/segment.hpp"
#include <map>
#include <vector>

/// @brief print a point
void print(point_s);

/// @brief print a segment
void print(segment_s);

/// @brief print a circle
void print(circle_s);

/// @brief print a set of points
void print(std::vector<point_s>);

/// @brief print a set of segments
void print(std::vector<segment_s>);

/// @brief insert or modify a key
/// @tparam Tkey the type of the key
/// @tparam Tval the type of the value
/// @param map reference to the map
/// @param key the key
/// @param value the value to assign
template <typename Tkey, typename Tval>
inline void insert_or_modify(std::map<Tkey, Tval> &map, Tkey key, Tval value) {
  if (!map.try_emplace(key, value).second) {
    map.at(key) = value;
  }
}
