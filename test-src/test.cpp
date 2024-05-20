#include "subzerolib/api/geometry/circle.hpp"
#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/geometry/segment.hpp"
#include "subzerolib/api/spline/catmull-rom.hpp"
#include "subzerolib/api/util/helper.hpp"

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

  CatmullRomSpline spline({{-1.5, 0.6}, {-0.6, 0.6}, {-0.2, 1.2}, {-0.6, 1.2}, {-1.5, 0}});
  spline.pad_velocity({1, 0}, {0, -1});
  std::vector<point_s> spline_points = spline.sample(100);

  for (auto &control : spline.get_control_points()) {
    file << "ctrl " << control.x << " " << control.y << std::endl;
  }

  for (auto &point : spline_points) {
    file << point.x << " " << point.y << std::endl;
  }

  std::cout << "Point count:" << spline_points.size() << std::endl;

  file.close();

  // TODO: write unit tests for line segment
  // TODO: write unit tests for circle/segment intersection

  segment_s segment{{-1, -1}, {1, 1}};
  circle_s circle{{0, 0}, 1};
  // should intersect at (-0.71, -0.71) and (0.71, 0.71)
  std::cout << "--- inputs ----------\n";
  print(segment);
  print(circle);
  std::cout << "--- intersections ---\n";
  print(circle.intersections(segment));

  circle.radius = 2;
  // should not intersect
  std::cout << "--- inputs ----------\n";
  print(segment);
  print(circle);
  std::cout << "--- intersections ---\n";
  print(circle.intersections(segment));

  circle.centre = point_s(0.5, 1);
  segment = segment_s{{-2, 0.75}, {0.25, 0.5}};
  // should intersect at 1 point
  std::cout << "--- inputs ----------\n";
  print(segment);
  print(circle);
  std::cout << "--- intersections ---\n";
  print(circle.intersections(segment));

  return 0;
}

// working through x drive motion limits
// ω_max = (wheel_max) / radius radians per second
// v_max = (wheel_max) * (sin(move_h - robor_h) + cos(move_h - robor_h))
// allowance after ω
// v_max = (wheel_max - ω * radius) * (sin(move_h - robor_h) + cos(move_h -
// robor_h))

// TODO: pure pursuit algorithm
/*
point_s carrot = spline_points[0];

loop {

circle_s seek_circle {{pose.x, pose.y}, lookahead};
// potential error: if the robot moves too fast with small enough lookahead,
// points may skip. interpolate?
while (spline_points.size() > 1 &&
seek_circle.contains(spline_points[1])) {
  spline_points.erase(spline_points.begin() + 1);
}

if (spline_points.size() == 1) { carrot = spline_points[0]; }
else {
  segment_s segment{spline_points[0], spline_points[1]};
  auto potentials = seek_circle.intersections(segment);
  if (potentials.size() == 2) {
    // get the one closer ot the start of the segment
    if (segment.start.dist(potentials[0]) < segment.start.dist(potentials[1])) {
      carrot = potentials[0];
    } else {
      carrot = potentials[1];
    }
  } else if (potentials.size() == 1) {
    carrot = potentials[0];
  } else {
    // keep the old carrot
  }
}

// move towards carrot with motion profile? or pid with slew

}
*/