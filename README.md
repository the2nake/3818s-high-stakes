# 3818 (~) High Stakes Codebase

## Description

A very work-in-progress codebase. `main` branch will stay nonfunctional until at least the end of the first week of July, at which time every test will be completed. The code will be cleaned up into a working version as soon as possible, then development will resume in a separate branch.

## Features

- Control code for asterisk and X drivetrain types
- Linear and angular balancing for opcontrol "arcarde" style movement
- Basic logging functionality
- Path generation using uniform Catmull-Rom spline
- Trapezoidal linear motion profile generation
- Generic PID implementation for holonomic drivetrains
- Custom odometry implementation for higher tracking accuracy
- Pure pursuit implementation
- Extensible state machine implementation
- Abstractions for gyroscopes and rotation encoders
- A variety of miscellaneous functions such as `shorter_turn` and `clamp_distance`

## Dependencies

### plot.py

`ffmpeg`

### Python (via pip)

`pros-cli matplotlib PyQt6`
