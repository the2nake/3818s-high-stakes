#pragma once
#include "subzerolib/api/geometry/pose.hpp"

/// @brief class that provides movement algorithms with position-based movement
/// functions
///
/// implementations of apporach_pose need not work as a move to pose function on
/// their own; they simply must approach correctly given that the pose is
/// roughly aligned with the direction of the drive
class ChassisController {
public:
  virtual void approach_pose(pose_s pose, double linv = std::nan("")) = 0;
  virtual void brake() = 0;
};