#include "subzerolib/api/trajectory/spline-trajectory.hpp"

#include "subzerolib/api/geometry/trajectory-point.hpp"
#include "subzerolib/api/spline/spline.hpp"
#include "subzerolib/api/trajectory/motion-profile/linear-motion-profile.hpp"
#include "subzerolib/api/util/search.hpp"

double SplineTrajectory::get_duration() { return vec.back().t; }
double SplineTrajectory::get_length() { return vec.back().s; }

trajectory_point_s SplineTrajectory::get_at_time(double t) {
  int next_i = binary_search<trajectory_point_s, double>(
      vec, t, [](trajectory_point_s p) -> double { return p.t; });
  if (next_i <= 0)
    return vec.front();
  if (next_i >= vec.size())
    return vec.back();
  double f = (t - vec[next_i - 1].t) / (vec[next_i].t - vec[next_i - 1].t);
  if (std::isnan(f) || f > 1.001 || f < -0.001) {
    return vec[next_i];
  }
  return lerp(vec[next_i - 1], vec[next_i], f);
}

trajectory_point_s SplineTrajectory::get_at_distance(double s) {
  int next_i = binary_search<trajectory_point_s, double>(
      vec, s, [](trajectory_point_s p) -> double { return p.s; });
  if (next_i <= 0)
    return vec.front();
  if (next_i >= vec.size())
    return vec.back();
  double f = (s - vec[next_i - 1].s) / (vec[next_i].s - vec[next_i - 1].s);
  if (std::isnan(f) || f > 1.001 || f < -0.001) {
    return vec[next_i];
  }
  return lerp(vec[next_i - 1], vec[next_i], f);
}

int SplineTrajectory::Builder::find_pose_index(pose_s pose) {
  double min_d = std::numeric_limits<double>::max();
  int return_index = 0;
  for (int i = 0; i < trajectory.size(); ++i) {
    double dist = pose.dist(trajectory[i]);
    if (dist < min_d) {
      return_index = i;
      min_d = dist;
    }
  }
  return return_index;
}

SplineTrajectory::Builder &
SplineTrajectory::Builder::with_spline(Spline *i_spline,
                                       std::vector<pose_s> control_points) {
  if (i_spline != nullptr) {
    b_spline = i_spline;
    b_control_points = std::move(control_points);
  }

  return *this;
}

SplineTrajectory::Builder &
SplineTrajectory::Builder::with_motion_profile(LinearMotionProfile *i_profile) {
  if (i_profile != nullptr) {
    b_profile = i_profile;
  }

  return *this;
}

SplineTrajectory::Builder &
SplineTrajectory::Builder::with_chassis(Chassis *i_chassis) {
  if (i_chassis != nullptr) {
    b_chassis = i_chassis;
  }

  return *this;
}

// TODO: refactor, split
std::shared_ptr<SplineTrajectory> SplineTrajectory::Builder::build() {
  // sample the spline
  auto spline_points = b_spline->sample_kinematics(sample_count);
  trajectory.resize(spline_points.size());
  for (int i = 0; i < spline_points.size(); ++i) {
    trajectory[i] = spline_points[i];
  }

  // get control indices
  std::vector<int> ctrl_indices;
  for (auto &ctrl_point : b_control_points) {
    int i = find_pose_index(ctrl_point);
    auto &traj_ctrl = trajectory[i];
    traj_ctrl.x = ctrl_point.x;
    traj_ctrl.y = ctrl_point.y;
    traj_ctrl.h = ctrl_point.h;
    ctrl_indices.push_back(i);
  }

  // apply motion profile
  trajectory.back().vx = 0;
  trajectory.back().vy = 0;
  b_profile->generate(trajectory.back().s);
  for (int i = 0; i < trajectory.size(); ++i) {
    auto lin_point = b_profile->get_point_at_distance(trajectory[i].s);
    trajectory[i].t = lin_point.t;

    if (i < trajectory.size() - 1) {
      auto &curr = trajectory[i];
      auto &next = trajectory[i + 1];

      double dx = next.x - curr.x;
      double dy = next.y - curr.y;
      double ds = hypot(dx, dy); // don't use s here

      curr.vx = lin_point.v / ds * dx;
      curr.vy = lin_point.v / ds * dy;
    }
  }

  // generate headings
  switch (b_mode) {
  case heading_mode_e::path:
    for (int i = 0; i < trajectory.size() - 1; ++i) {
      auto &curr = trajectory[i];
      auto &next = trajectory[i + 1];
      curr.h = 90.0 - in_deg(std::atan2(next.y - curr.y, next.x - curr.x));
    }
    break;
  case heading_mode_e::pose:
    for (int i = 0; i < trajectory.size(); ++i) {
      bool not_ctrl_point =
          std::find(ctrl_indices.begin(), ctrl_indices.end(), i) ==
          ctrl_indices.end();
      if (not_ctrl_point) {
        int i0 = 0, i1 = 0;
        for (int j = 1; j < ctrl_indices.size(); ++j) {
          if (ctrl_indices[j] > i) {
            i0 = ctrl_indices[j - 1];
            i1 = ctrl_indices[j];
            break;
          }
        }
        double s_pct = (trajectory[i].s - trajectory[i0].s) /
                       (trajectory[i1].s - trajectory[i0].s);
        double h0 = trajectory[i0].h, h1 = trajectory[i1].h;
        trajectory[i].h = h0 + s_pct * shorter_turn(h0, h1, 360.0);
      }
    }
    break;
  }

  // generate vh
  trajectory.back().vh = 0;
  for (int i = 0; i < trajectory.size() - 1; ++i) {
    auto &curr = trajectory[i];
    auto &next = trajectory[i + 1];

    double dt = next.t - curr.t;
    trajectory[i].vh = shorter_turn(curr.h, next.h, 360.0) / dt;
  }

  // limit based on model-given velocities
  std::vector<double> max_vels = b_chassis->get_wheel_max();
  // used to reformulate time points
  std::vector<double> dts(trajectory.size());
  dts.front() = 0.0;
  for (int i = 1; i < trajectory.size(); ++i) {
    auto &traj_p = trajectory[i];

    // generate wheel velocities
    point_s local_v = rotate_acw(traj_p.vx, traj_p.vy, traj_p.h);
    auto wheel_vels =
        b_chassis->get_wheel_vels(local_v.x, local_v.y, in_rad(traj_p.vh));

    // vel_scale used to accumulate scaling ratios
    double vel_scale = 1.0;
    for (int j = 0; j < wheel_vels.size(); ++j) {
      double mag = std::abs(wheel_vels[j]);
      double max = std::abs(max_vels[j]);

      // scale uniformly if exceeding
      if (mag > max) {
        double scale = max / mag;
        for (auto &v : wheel_vels) {
          v *= scale;
        }
        vel_scale *= 0.999999 * scale;
      }
    }
    // scale time and store
    double dt = trajectory[i].t - trajectory[i - 1].t;
    dts[i] = dt / vel_scale;
    // scale velocities to match wheel velocity changes
    traj_p.vx *= vel_scale;
    traj_p.vy *= vel_scale;
    traj_p.vh *= vel_scale;
  }

  // integrate new time deltas
  for (int i = 1; i < trajectory.size(); ++i) {
    trajectory[i].t = trajectory[i - 1].t + dts[i];
  }

  std::shared_ptr<SplineTrajectory> ptr{new SplineTrajectory()};
  ptr->vec = trajectory;

  return ptr;
}
