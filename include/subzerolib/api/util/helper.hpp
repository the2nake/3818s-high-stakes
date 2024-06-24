#pragma once

#include "subzerolib/api/geometry/circle.hpp"
#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/geometry/segment.hpp"
#include <map>
#include <vector>

void print(point_s);
void print(segment_s);
void print(circle_s);

void print(std::vector<point_s>);
void print(std::vector<segment_s>);

// TODO: delete all instances of "not" keyword

template <typename Tkey, typename Tval>
void insert_or_modify(std::map<Tkey, Tval> &map, Tkey key, Tval value) {
  if (!map.try_emplace(key, value).second) {
    map.at(key) = value;
  }
}
