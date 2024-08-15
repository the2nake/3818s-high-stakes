# 3818 (~) High Stakes Codebase

## Description

The codebase for a certain team of 3818, consisting of 4 members. The `main` branch should always be functional.

## Features

- Control code for asterisk and X drivetrain types
- Linear and angular balancing for opcontrol "arcarde" style movement
- Basic logging functionality
- Path generation using uniform Catmull-Rom spline
- Trapezoidal linear motion profile generation
- Generic PID implementation for holonomic drivetrains
- Custom odometry implementation for higher tracking accuracy
- Sensor fusion with 2 IMUs with the KFOdometry class
- Pure pursuit implementation
- Extensible state machine implementation
- Abstractions for gyroscopes and rotation encoders
- Filtered gyroscope class using modular arithmetic averages
- A variety of miscellaneous functions such as `shorter_turn` and `clamp_distance`

## Dependencies

### plot.py

`ffmpeg`

### Python (via pip)

`pros-cli matplotlib PyQt6`
