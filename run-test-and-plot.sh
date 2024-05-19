#!/usr/bin/env bash

g++ test-src/test.cpp src/subzerolib/api/geometry/point.cpp src/subzerolib/api/spline/catmull-rom.cpp -Iinclude && ./a.out && python3 test-src/plot.py