#!/usr/bin/env bash

g++ test-src/test.cpp src/subzerolib/api/geometry/point.cpp src/subzerolib/api/geometry/segment.cpp src/subzerolib/api/geometry/circle.cpp src/subzerolib/api/util/helper.cpp src/subzerolib/api/spline/catmull-rom.cpp -Iinclude
./a.out