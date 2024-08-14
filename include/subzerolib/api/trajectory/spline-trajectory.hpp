#pragma once

#include "subzerolib/api/chassis/chassis.hpp"
#include "subzerolib/api/geometry/trajectory-point.hpp"
#include "subzerolib/api/spline/spline.hpp"
#include "subzerolib/api/trajectory/motion-profile/linear-motion-profile.hpp"
#include "subzerolib/api/trajectory/trajectory.hpp"

#include "subzerolib/api/geometry/pose.hpp"

#include <memory>
#include <vector>

class SplineTrajectory : public Trajectory {
public:
  enum class heading_mode_e { path, pose };

  double get_duration() override;
  double get_length() override;

  trajectory_point_s get_at_time(double t) override;
  trajectory_point_s get_at_distance(double s) override;

  void print() {
    for (auto &p : vec) {
      printf("t=%6.3f s=%6.3f (%5.2f,%5.2f) h=%6.1f vh=%4.0f vx=%5.2f vy=%5.2f "
             "v=%5.2f\n",
             p.t,
             p.s,
             p.x,
             p.y,
             p.h,
             p.vh,
             p.vx,
             p.vy,
             std::hypot(p.vx, p.vy));
    }
  }

private:
  SplineTrajectory() {}

  std::vector<trajectory_point_s> vec;

public:
  class Builder {
  public:
    Builder(heading_mode_e i_mode, int i_sample_count)
        : b_mode(i_mode), sample_count(i_sample_count) {}

    Builder &with_spline(Spline *i_spline,
                         std::vector<pose_s> i_control_points);
    Builder &with_motion_profile(LinearMotionProfile *profile);
    Builder &with_chassis(Chassis *i_chassis);

    std::shared_ptr<SplineTrajectory> build();

    const heading_mode_e b_mode;
    const int sample_count;

  private:
    int find_pose_index(pose_s pose);

    Spline *b_spline = nullptr;
    LinearMotionProfile *b_profile = nullptr;
    Chassis *b_chassis = nullptr;

    std::vector<pose_s> b_control_points;
    std::vector<trajectory_point_s> trajectory;
  };
};
