#include "subzerolib/api/util/helper.hpp"

#include <stdio.h>

void print(point_s point) {
  printf("point_s: (%.2f, %.2f)\n", point.x, point.y);
}

void print(segment_s segment) {
  printf("segment_s: start (%.2f, %.2f), end (%.2f, %.2f)\n", segment.start.x,
         segment.start.y, segment.end.x, segment.end.y);
}

void print(circle_s circle) {
  printf("circle_s: centre(%.2f, %.2f), radius %.2f\n", circle.centre.x,
         circle.centre.y, circle.radius);
}

void print(std::vector<point_s> points) {
  for (auto &point : points) {
    print(point);
  }

  if (points.empty()) {
    printf("std::vector<point_s> empty\n");
  }
}

void print(std::vector<segment_s> segments) {
  for (auto &segment : segments) {
    print(segment);
  }

  if (segments.empty()) {
    printf("std::vector<segment_s> empty\n");
  }
}
