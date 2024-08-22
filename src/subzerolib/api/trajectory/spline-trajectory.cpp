#include "subzerolib/api/trajectory/spline-trajectory.hpp"

#include "subzerolib/api/geometry/circle.hpp"
#include "subzerolib/api/geometry/segment.hpp"
#include "subzerolib/api/geometry/trajectory-point.hpp"
#include "subzerolib/api/spline/spline.hpp"
#include "subzerolib/api/trajectory/motion-profile/linear-motion-profile.hpp"
#include "subzerolib/api/util/math.hpp"
#include "subzerolib/api/util/search.hpp"
#include <limits>

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
  if (std::isnan(f)) {
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
  if (std::isnan(f)) {
    return vec[next_i];
  }
  return lerp(vec[next_i - 1], vec[next_i], f);
}

int SplineTrajectory::Builder::find_pose_index(pose_s pose) {
  double min_d = std::numeric_limits<double>::max();
  int return_index = 0;
  for (int i = 0; i < traj.size(); ++i) {
    double dist = pose.dist(traj[i]);
    if (dist < min_d) {
      return_index = i;
      min_d = dist;
    }
  }
  return return_index;
}

SplineTrajectory::Builder &
SplineTrajectory::Builder::with_spline(std::shared_ptr<Spline> i_spline,
                                       std::vector<pose_s> i_control_points) {
  if (i_spline != nullptr) {
    spline = i_spline;
    control_points = std::move(i_control_points);
  }

  return *this;
}

SplineTrajectory::Builder &SplineTrajectory::Builder::with_motion_profile(
    std::shared_ptr<LinearMotionProfile> i_profile) {
  if (i_profile != nullptr) {
    profile = i_profile;
  }

  return *this;
}

SplineTrajectory::Builder &
SplineTrajectory::Builder::with_chassis(std::shared_ptr<Chassis> i_chassis) {
  if (i_chassis != nullptr) {
    chassis = i_chassis;
  }

  return *this;
}

void SplineTrajectory::Builder::sample_spline() {
  spline_points = spline->sample_kinematics(sample_count);
  traj.resize(spline_points.size());
  for (int i = 0; i < spline_points.size(); ++i) {
    traj[i] = spline_points[i];
  }
}

void SplineTrajectory::Builder::get_control_indices() {
  for (auto &ctrl_point : control_points) {
    int i = find_pose_index(ctrl_point);
    auto &traj_ctrl = traj[i];
    traj_ctrl.x = ctrl_point.x;
    traj_ctrl.y = ctrl_point.y;
    traj_ctrl.h = ctrl_point.h;
    ctrl_is.push_back(i);
  }
}

void SplineTrajectory::Builder::apply_motion_profile() {
  traj.back().vx = 0;
  traj.back().vy = 0;
  profile->generate(traj.back().s);
  for (int i = 0; i < traj.size(); ++i) {
    auto lin_point = profile->get_point_at_distance(traj[i].s);
    traj[i].t = lin_point.t;

    if (i < traj.size() - 1) {
      auto &curr = traj[i];
      auto &next = traj[i + 1];

      double dx = next.x - curr.x;
      double dy = next.y - curr.y;
      double ds = hypot(dx, dy); // don't use s here

      curr.vx = lin_point.v / ds * dx;
      curr.vy = lin_point.v / ds * dy;
    }
  }
}

std::pair<double, double> get_a(double v0, double v, double dt, double accel) {
  return {(-v0 + sqrt(v0 * v0 + 4 * accel * dt * v)) / (2 * accel * dt),
          (-v0 - sqrt(v0 * v0 + 4 * accel * dt * v)) / (2 * accel * dt)};
}

// TODO:
void SplineTrajectory::Builder::constrain_2d_accel() {
  // rely on time paramterization
  // for each 2d accel that's wrong,
  //   transform everything to dt space
  //   replace time and velocities such that the delta is correct
  //     for some factor a, dt -> a * dt, v = v / a
  //     if accel = dv / dt, how to get a factor?
  //     (a [vx vy] - [v0x v0y]) / dt = m_accel
  double m_accel = profile->get_max_accel();

  std::vector<double> dts(traj.size());
  dts[0] = 0;
  for (int i = 1; i < dts.size(); ++i) {
    dts[i] = traj[i].t - traj[i - 1].t;
  }

  // naive method, inefficient
  for (int i = 1; i < traj.size(); ++i) {
    double accel = std::hypot((traj[i].vx - traj[i - 1].vx) / (dts[i]),
                              (traj[i].vy - traj[i - 1].vy) / (dts[i]));

    // draw a circle around the vector
    // scale the vx and vy to hit that circle
    if (accel > m_accel) {
      point_s center{traj[i - 1].vx, traj[i - 1].vy};
      circle_s possible{center, m_accel * dts[i]};
      segment_s vel_line{
          {-2 * traj[i].vx, -2 * traj[i].vy},
          { 2 * traj[i].vx,  2 * traj[i].vy}
      };
      auto intersections = possible.intersections(vel_line);
      point_s new_vel;
      double max_dist = std::numeric_limits<double>::min();
      for (auto &inter : intersections) {
        auto dist = inter.dist({0, 0});
        if (dist > max_dist) {
          max_dist = dist;
          new_vel = inter;
        }
      }
      double scale =
          segment_s{
              {0, 0},
              new_vel
      }
              .length() /
          vel_line.length();
      traj[i].vx *= scale;
      traj[i].vy *= scale;
      traj[i].vh *= scale;
      dts[i + 1] /= scale;
    }
  }

  for (int i = 1; i < traj.size(); ++i) {
    traj[i].t = dts[i] + traj[i - 1].t;
  }
}

bool SplineTrajectory::Builder::is_accel_broken(int i) {
  double m_accel = profile->get_max_accel();
  double m_decel = profile->get_max_decel();
  if (i >= 0 && i < traj.size()) {
    double accel = get_accel(i);
    return accel > m_accel || accel < -m_decel;
  }
  bool output = false;
  for (int i = 1; i < traj.size(); ++i) {
    if (is_accel_broken(i)) {
      printf("[%d] %6.2f\n", i, get_accel(i));
      output = true;
    }
  }
  return output;
}

double SplineTrajectory::Builder::get_accel(int i) {
  clamp<int>(i, 1, traj.size() - 1);
  auto point = traj[i];
  auto prev = traj[i - 1];
  double accel_x = (point.vx - prev.vx) / (point.t - prev.t);
  double accel_y = (point.vx - prev.vx) / (point.t - prev.t);
  return std::hypot(accel_x, accel_y);
}

void SplineTrajectory::Builder::generate_heading() {
  if (b_mode == heading_mode_e::path) {
    for (int i = 0; i < traj.size() - 1; ++i) {
      auto &curr = traj[i];
      auto &next = traj[i + 1];
      curr.h = 90.0 - in_deg(std::atan2(next.y - curr.y, next.x - curr.x));
    }
    return;
  }

  int j = 0, i0 = 0, i1 = 0;
  double dh = 0, dt = 0, max_a = 0.0;
  for (int i = 0; i < traj.size(); ++i) {
    bool is_ctrl =
        std::find(ctrl_is.begin(), ctrl_is.end(), i) != ctrl_is.end();
    if (is_ctrl) {
      if (j++ < ctrl_is.size()) {
        i0 = ctrl_is[j - 1];
        i1 = ctrl_is[j];
        continue;
      }
      break;
    }
    dh = shorter_turn(traj[i0].h, traj[i1].h, 360.0);
    dt = traj[i1].t - traj[i0].t;
    max_a = 4 * dh / (dt * dt);

    double t_i = traj[i].t - traj[i0].t;
    traj[i].h = traj[i0].h;
    if (t_i < dt / 2.0) {
      traj[i].h += max_a * t_i * t_i * 0.5;
    } else {
      t_i -= dt / 2.0;
      traj[i].h += 0.5 * dh + (2 * dh * t_i / dt) - (0.5 * max_a * t_i * t_i);
    }

    if (i >= 1) {
      traj[i - 1].vh = shorter_turn(traj[i - 1].h, traj[i].h, 360.0) /
                       (traj[i].t - traj[i - 1].t);
    }
  }
}

void SplineTrajectory::Builder::apply_model_constraints() {
  std::vector<double> v_max = chassis->get_wheel_max();
  // used to reformulate time points
  std::vector<double> dts(traj.size());
  dts.front() = 0.0;
  for (int i = 1; i < traj.size(); ++i) {
    auto &p = traj[i];

    point_s local = rotate_acw(p.vx, p.vy, p.h);
    auto v_wheel = chassis->get_wheel_vels(local.x, local.y, in_rad(p.vh));

    // used to accumulate scaling ratios
    double vel_scale = 1.0;
    for (int j = 0; j < v_wheel.size(); ++j) {
      double mag = std::abs(v_wheel[j]);
      double max = std::abs(v_max[j]);

      // scale uniformly if exceeding
      if (mag > max) {
        double scale = max / mag;
        for (auto &v : v_wheel) {
          v *= scale;
        }
        vel_scale *= 0.999999 * scale;
      }
    }
    double dt = traj[i].t - traj[i - 1].t;
    dts[i] = dt / vel_scale;
    p.vx *= vel_scale;
    p.vy *= vel_scale;
    p.vh *= vel_scale;
  }

  // integrate new time deltas
  for (int i = 1; i < traj.size(); ++i) {
    traj[i].t = traj[i - 1].t + dts[i];
  }
}

std::shared_ptr<SplineTrajectory> SplineTrajectory::Builder::build() {
  sample_spline();
  get_control_indices();

  apply_motion_profile();

  // constrain_2d_accel();

  generate_heading();

  apply_model_constraints();

  constrain_2d_accel();
  is_accel_broken();

  std::shared_ptr<SplineTrajectory> ptr{new SplineTrajectory()};
  ptr->vec = traj;

  return ptr;
}
