#pragma once

#include "subzerolib/api/geometry/point.hpp"
#include <cmath>

const double K_PI = 3.141592654;
const double K_EPSILON = 0.00001;
const double K_SQRT_2 = 1.414213562;

double rougheq(double a, double b);
double in_rad(double deg);
double in_deg(double rad);

/// @brief calculates a solely positive modulus
/// @param a the first argument
/// @param circ the cycle amount
/// @returns 0 <= x < circ
double mod(double a, double circ);

/// @brief finds the shortest turn from h0 to hf.
/// @param h0 initial angle
/// @param hf final angle
/// @param circle_size number of units in a circle. degrees default (360.0)
/// @returns the difference in angle, with (+) values clockwise.
template <typename T>
auto shorter_turn(T h0, T hf, T circle_size = 360.0) -> decltype(hf - h0) {
  auto rightward_angle = mod(hf - h0, circle_size);
  if (std::abs(rightward_angle) < std::abs(circle_size / 2.0)) {
    return rightward_angle;
  } else {
    return rightward_angle - circle_size;
  }
}

/// @brief linearly interpolates between two values
/// @param a first value
/// @param b second value
/// @param t the interpolation factor (0 <= t <= 1)
/// @returns interpolated value between a and b
template <typename T, typename T2>
auto lerp(T a, T b, T2 t) -> decltype(a * t) {
  return a + (b - a) * t;
}

template <typename T> void clamp(T &val, T min, T max) {
  if (max < min) {
    std::swap(max, min);
  }
  if (max < val) {
    val = max;
  } else if (min > val) {
    val = min;
  }
}

template <typename T> void clamp_distance(T dist, T &x, T &y) {
  double d = std::hypot(x, y);
  if (std::abs(d) > dist) {
    double sin = y / d;
    double cos = x / d;
    d = std::min(dist, d);
    y = sin * d;
    x = cos * d;
  }
}

template <typename T> point_s rotate_acw(T x, T y, T h) {
  double rad = in_rad(-h);
  return {x * cos(rad) + y * sin(rad), y * cos(rad) - x * sin(rad)};
}
