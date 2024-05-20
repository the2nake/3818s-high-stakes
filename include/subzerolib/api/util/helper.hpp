#pragma once

#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/geometry/segment.hpp"
#include "subzerolib/api/geometry/circle.hpp"
#include <vector>

void print(point_s);
void print(segment_s);
void print(circle_s);

void print(std::vector<point_s>);
void print(std::vector<segment_s>);