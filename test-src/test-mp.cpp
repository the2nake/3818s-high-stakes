#include "subzerolib/api/spline/catmull-rom.hpp"
#include "subzerolib/api/spline/spline.hpp"
#include "subzerolib/api/trajectory/motion-profile/linear-motion-profile.hpp"
#include "subzerolib/api/trajectory/motion-profile/trapezoidal-motion-profile.hpp"
#include "subzerolib/api/util/math.hpp"

#include <cmath>
#include <limits>

struct trajectory_point_s {
  trajectory_point_s(double i_t = 0,
                     double i_s = 0,
                     double i_x = 0,
                     double i_vx = 0,
                     double i_y = 0,
                     double i_vy = 0,
                     double i_h = 0,
                     double i_vh = 0)
      : t(i_t), s(i_s), x(i_x), vx(i_vx), y(i_y), vy(i_vy), h(i_h), vh(i_vh) {}
  trajectory_point_s(spline_point_s &point)
      : s(point.s), x(point.x), vx(point.vx), y(point.y), vy(point.vy) {}

  double t = 0;
  double s = 0;
  double x = 0;
  double vx = 0;
  double y = 0;
  double vy = 0;
  double h = 0;
  double vh = 0;
};

int find_pose_index(std::vector<spline_point_s> &vec, pose_s pose) {
  double min_d = std::numeric_limits<double>::max();
  double index = 0.0;
  for (int i = 0; i < vec.size(); ++i) {
    double dist = pose.dist(vec[i].point());
    if (dist < min_d) {
      index = i;
      min_d = dist;
    }
  }
  double dx = pose.x - vec[index].x;
  double dy = pose.y - vec[index].y;
  bool same_dir = vec[index].vx * dx + dy * vec[index].vy > 0;
  return index + int(same_dir);
}

int main() {
  // set up a linear motion profile
  LinearMotionProfile *generator = new TrapezoidalMotionProfile{1.85, 6, 6};
  generator->set_resolution(0.01);

  // generate the curve using a catmull rom spline
  std::vector<pose_s> ctrl_points = {
      pose_s{  0.0,  0.0,   0.0},
      pose_s{  0.4,  0.6,  45.0},
      pose_s{ -0.2,  0.6,  60.0},
      pose_s{-0.75, 0.75, -45.0}
  };
  CatmullRomSpline spline(ctrl_points);
  spline.pad_velocity({0.5, 0.5}, {-0.25, 0.25});
  std::vector<spline_point_s> sampled_points = spline.sample_kinematics(300);
  std::vector<point_s> raw_path = spline.sample_coordinates(300);

  auto final = interpolate_heading(raw_path, ctrl_points);
  for (auto p : final) {
    printf("(%f, %f) h=%f\n", p.x, p.y, p.h);
  }

  std::vector<trajectory_point_s> generated_profile(sampled_points.size());
  for (int i = 0; i < sampled_points.size(); ++i) {
    generated_profile[i] = sampled_points[i];
  }
  std::vector<int> ctrl_indices;
  for (auto &c : ctrl_points) {
    int i = find_pose_index(sampled_points, c);
    generated_profile[i].x = c.x;
    generated_profile[i].y = c.y;
    generated_profile[i].h = c.h;
    ctrl_indices.push_back(i);
  }

  // apply profile to path
  generator->generate(generated_profile.back().s);
  // dynamic_cast<TrapezoidalMotionProfile *>(generator)->print();
  for (int i = 0; i < generated_profile.size(); ++i) {
    LinearMotionProfile::profile_point_s motion =
        generator->get_point_at_distance(generated_profile[i].s);
    generated_profile[i].t = motion.t;
    if (i < generated_profile.size() - 1) {
      double dx = generated_profile[i + 1].x - generated_profile[i].x;
      double dy = generated_profile[i + 1].y - generated_profile[i].y;
      double ds = hypot(dx, dy); // don't use s here
      generated_profile[i].vx = motion.v / ds * dx;
      generated_profile[i].vy = motion.v / ds * dy;
    }
  }
  generated_profile.back().vx = 0;
  generated_profile.back().vy = 0;

  // lerp headings and angular velocity
  for (int i = 0; i < generated_profile.size(); ++i) {
    if (std::find(ctrl_indices.begin(), ctrl_indices.end(), i) ==
        ctrl_indices.end()) {
      int start_index = 0;
      int end_index = 0;
      for (int j = 1; j < ctrl_indices.size(); ++j) {
        if (ctrl_indices[j] > i) {
          start_index = ctrl_indices[j - 1];
          end_index = ctrl_indices[j];
          break;
        }
      }
      double factor =
          (generated_profile[i].s - generated_profile[start_index].s) /
          (generated_profile[end_index].s - generated_profile[start_index].s);
      double h0 = generated_profile[start_index].h;
      double h1 = generated_profile[end_index].h;
      generated_profile[i].h = h0 + factor * shorter_turn(h0, h1, 360.0);
    }
  }
  generated_profile.back().vh = 0;

  // generate vh
  for (int i = 0; i < generated_profile.size() - 1; ++i) {
    double dt = generated_profile[i + 1].t - generated_profile[i].t;
    generated_profile[i].vh = shorter_turn(generated_profile[i].h,
                                           generated_profile[i + 1].h,
                                           360.0) /
                              dt;
  }

  // show the profile
  for (trajectory_point_s &p : generated_profile) {
    printf("t=%.3f s=%.3f (%.2f, %.2f) h=%.3f vh=%.1f vx=%.2f vy=%.2f v=%.2f\n",
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

  // angular velocity contribution formulae
  // if o = radians/second cw
  // lf = radius * o
  // lm = radius * o * (0.5 * sqrt2) * (adjustment for radii)
  // lb = radius * o
  // rf = -radius * o
  // rm = -radius * o * (0.5 * sqrt2) * (adjustment for radii)
  // rb = -radius * o
  //
  // linear velocity contribution formula
  // if v = meters/second (split as vector components vx, vy after rotation to
  // local frame)
  // lf = (0.5 * sqrt2) * (vx + vy)
  // lm = vy
  // lb = (0.5 * sqrt2) * (-vx + vy)
  // rf = (0.5 * sqrt2) * (-vx + vy)
  // rm = vy
  // rb = (0.5 * sqrt2) * (vx + vy)
  //
  // if scaling v and o the same amount, you will get the appropriate limited
  // values so, just set a "maximum value" for each generated velocity from
  // chassis model then scale
  //
  // std::vector<double> wheel_vels = Chassis::get_wheel_vels(v, h, o);
  // for (int i = 0; i < wheel_vels.size(); ++i) {
  //   auto &vel = wheel_vels[i];
  //   double max_vel = Chassis::get_wheel_max_vel(i);
  //   if (std::abs(vel) > std::abs(max_vel)) {
  //     double scale = std::abs(max_vel) / std::abs(vel);
  //     for (auto &v : wheel_vels) {
  //       v = v * scale;
  //     }
  //   }
  // }

  // TODO: clamp with model constraints
  // TODO: clamp with kinematic constraints
  // TODO: unit test with integration
  return 0;
}
