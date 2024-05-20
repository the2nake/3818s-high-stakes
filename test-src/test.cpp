#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/spline/catmull-rom.hpp"
#include <bits/stdc++.h>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

int main() {
  // Creating a directory
  mkdir("test-output", 0777);
  std::fstream file;
  file.open("test-output/output.txt", std::fstream::out);
  file.clear();

  CatmullRomSpline spline(
      {{1.5, 0}, {2.4, 2.4}, {0, 2.4}, {0, 0.5}, {2, 1}, {0.5, 1.5}});
  spline.pad_velocity({0, 2}, {-2, 0});
  std::vector<point_s> points = spline.sample(100);

  for (auto &control : spline.get_control_points()) {
    file << "ctrl " << control.x << " " << control.y << std::endl;
  }

  for (auto &point : points) {
    file << point.x << " " << point.y << std::endl;
  }

  std::cout << "Point count:" << points.size() << std::endl;

  file.close();

  return 0;
}

// working through x drive motion limits
// ω_max = (wheel_max) / radius radians per second
// v_max = (wheel_max) * (sin(move_h - robor_h) + cos(move_h - robor_h))
// allowance after ω
// v_max = (wheel_max - ω * radius) * (sin(move_h - robor_h) + cos(move_h - robor_h))

// TODO: pure pursuit