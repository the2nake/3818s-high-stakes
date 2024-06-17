#pragma once
#include "subzerolib/api/geometry/pose.hpp"

/// @brief f
class ChassisController {
public:
  virtual void approach_pose(pose_s pose, double linv = std::nan("")) = 0;
  virtual void brake() = 0;
};