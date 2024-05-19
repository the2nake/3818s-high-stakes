#include "subzerolib/api/geometry/point.hpp"

point_s operator+(point_s a, point_s b) { return {a.x + b.x, a.y + b.y}; }
point_s operator-(point_s a, point_s b) { return {a.x - b.x, a.y - b.y}; }