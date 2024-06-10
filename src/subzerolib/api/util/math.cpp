#include "subzerolib/api/util/math.hpp"

double in_rad(double deg) {
  return deg * K_PI / 180.0;
}

double in_deg(double rad) {
  return 180.0 * rad / K_PI;
}
