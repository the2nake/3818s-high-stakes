#pragma once

/// @brief linearly interpolates between two values
/// @param a first value
/// @param b second value
/// @param t the interpolation factor (0 <= t <= 1)
/// @returns interpolated value between a and b
template <typename T, typename T2>
T lerp(T a, T b, T2 t) {
  return a + (b - a) * t;
}
