#include "subzerolib/api/util/math.hpp"

inline double in_rad(double deg) { return deg * K_PI / 180.0; }

inline double in_deg(double rad) { return 180.0 * rad / K_PI; }

double mod(double x, double modulo) {
  if (modulo == 0) {
    return x;
  }

  if (x < 0) {
    return mod(x + modulo, modulo);
  }

  if (x >= modulo) {
    return mod(x - modulo, modulo);
  }

  return x;
}