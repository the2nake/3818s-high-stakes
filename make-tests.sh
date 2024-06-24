#!/usr/bin/env bash
g++ test-src/test-utils.cpp src/subzerolib/api/util/controls.cpp src/subzerolib/api/util/math.cpp -Iinclude -o test-output/test-utils.bin
g++ test-src/test-plot.cpp src/subzerolib/api/geometry/point.cpp src/subzerolib/api/geometry/segment.cpp src/subzerolib/api/geometry/circle.cpp src/subzerolib/api/util/helper.cpp src/subzerolib/api/spline/catmull-rom.cpp -Iinclude -DEIEN_DONT_VECTORIZE -o test-output/test-plot.bin
