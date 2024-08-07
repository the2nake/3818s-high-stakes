#include "subzerolib/api/spline/catmull-rom.hpp"
#include "subzerolib/api/spline/spline.hpp"
#include "subzerolib/api/trajectory/motion-profile/linear-motion-profile.hpp"
#include "subzerolib/api/trajectory/motion-profile/trapezoidal-motion-profile.hpp"
#include "subzerolib/api/util/math.hpp"

#include <cmath>
#include <limits>

void show_vector(std::vector<double> vector) {
  for (auto &i : vector) {
    printf("%f\n", i);
  }
}

class StarChassisKinematics {
public:
  StarChassisKinematics(double i_max_vel, double i_boost, double i_corner)
      : max_vel(std::abs(i_max_vel)), boost_radius(std::abs(i_boost)),
        corner_radius(std::abs(i_corner)) {}
  std::vector<double> get_wheel_max() {
    return {0.5 * K_SQRT_2 * max_vel,
            max_vel,
            0.5 * K_SQRT_2 * max_vel,
            0.5 * K_SQRT_2 * max_vel,
            max_vel,
            0.5 * K_SQRT_2 * max_vel};
  }
  std::vector<double> get_wheel_vels(double vx, double vy, double v_ang) {
    double angular_components[] = {corner_radius * v_ang,
                                   boost_radius * v_ang,
                                   corner_radius * v_ang,
                                   -corner_radius * v_ang,
                                   -boost_radius * v_ang,
                                   corner_radius * v_ang};
    double linear_components[] = {(0.5 * K_SQRT_2) * (vx + vy),
                                  vy,
                                  (0.5 * K_SQRT_2) * (-vx + vy),
                                  (0.5 * K_SQRT_2) * (-vx + vy),
                                  vy,
                                  (0.5 * K_SQRT_2) * (vx + vy)};
    std::vector<double> final_vels(6);
    for (int i = 0; i < final_vels.size(); ++i) {
      final_vels[i] = linear_components[i] + angular_components[i];
    }
    return final_vels;
  }

private:
  double max_vel = 1;
  double boost_radius = 1;
  double corner_radius = 1;
};

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
  int return_index = 0;
  for (int i = 0; i < vec.size(); ++i) {
    double dist = pose.dist(vec[i].point());
    if (dist < min_d) {
      return_index = i;
      min_d = dist;
    }
  }
  return return_index;
}

int main() {
  // set up a linear motion profile
  StarChassisKinematics kinematics(1.73, 0.35, 0.37);
  LinearMotionProfile *generator = new TrapezoidalMotionProfile{1.73, 4, 4};
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
  std::vector<spline_point_s> sampled_points = spline.sample_kinematics(400);

  std::vector<trajectory_point_s> trajectory(sampled_points.size());
  for (int i = 0; i < sampled_points.size(); ++i) {
    trajectory[i] = sampled_points[i];
  }
  std::vector<int> ctrl_indices;
  for (auto &c : ctrl_points) {
    int i = find_pose_index(sampled_points, c);
    trajectory[i].x = c.x;
    trajectory[i].y = c.y;
    trajectory[i].h = c.h;
    ctrl_indices.push_back(i);
  }

  // apply profile to path
  generator->generate(trajectory.back().s);
  for (int i = 0; i < trajectory.size(); ++i) {
    LinearMotionProfile::profile_point_s motion =
        generator->get_point_at_distance(trajectory[i].s);
    trajectory[i].t = motion.t;
    if (i < trajectory.size() - 1) {
      double dx = trajectory[i + 1].x - trajectory[i].x;
      double dy = trajectory[i + 1].y - trajectory[i].y;
      double ds = hypot(dx, dy); // don't use s here
      trajectory[i].vx = motion.v / ds * dx;
      trajectory[i].vy = motion.v / ds * dy;
    }
  }
  trajectory.back().vx = 0;
  trajectory.back().vy = 0;

  // lerp headings and angular velocity
  for (int i = 0; i < trajectory.size(); ++i) {
    if (std::find(ctrl_indices.begin(), ctrl_indices.end(), i) ==
        ctrl_indices.end()) {
      int i0 = 0;
      int i1 = 0;
      for (int j = 1; j < ctrl_indices.size(); ++j) {
        if (ctrl_indices[j] > i) {
          i0 = ctrl_indices[j - 1];
          i1 = ctrl_indices[j];
          break;
        }
      }
      double factor = (trajectory[i].s - trajectory[i0].s) /
                      (trajectory[i1].s - trajectory[i0].s);
      double h0 = trajectory[i0].h;
      double h1 = trajectory[i1].h;
      trajectory[i].h = h0 + factor * shorter_turn(h0, h1, 360.0);
    }
  }

  // generate vh
  for (int i = 0; i < trajectory.size() - 1; ++i) {
    double dt = trajectory[i + 1].t - trajectory[i].t;
    trajectory[i].vh =
        shorter_turn(trajectory[i].h, trajectory[i + 1].h, 360.0) / dt;
  }
  trajectory.back().vh = 0;

  // limit based on model-given velocities
  std::vector<double> max_vels = kinematics.get_wheel_max();
  // used to reformulate time points
  std::vector<double> dts(trajectory.size());

  dts.front() = 0.0;
  for (int i = 1; i < trajectory.size(); ++i) {
    auto &traj_p = trajectory[i];
    point_s local_v = rotate_acw(traj_p.vx, traj_p.vy, traj_p.h);
    auto wheel_vels =
        kinematics.get_wheel_vels(local_v.x, local_v.y, in_rad(traj_p.vh));

    double vel_scale = 1.0;
    for (int j = 0; j < wheel_vels.size(); ++j) {
      double mag = std::abs(wheel_vels[j]);
      double max = std::abs(max_vels[j]);

      if (mag > max) {
        double scale = max / mag;
        for (auto &v : wheel_vels) {
          v *= scale;
        }
        vel_scale *= 0.999999 * scale;
      }
    }

    double dt = trajectory[i].t - trajectory[i - 1].t;
    dts[i] = dt / vel_scale;
    traj_p.vx *= vel_scale;
    traj_p.vy *= vel_scale;
    traj_p.vh *= vel_scale;
  }

  for (int i = 1; i < trajectory.size(); ++i) {
    trajectory[i].t = trajectory[i - 1].t + dts[i];
  }

  // show the profile
  for (trajectory_point_s &p : trajectory) {
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

  // check for unmet model constraints
  std::vector<double> maxs = kinematics.get_wheel_max();
  for (trajectory_point_s &p : trajectory) {
    auto loc = rotate_acw(p.vx, p.vy, p.h);
    auto vels = kinematics.get_wheel_vels(loc.x, loc.y, in_rad(p.vh));
    bool broken = false;
    for (int i = 0; i < vels.size(); ++i) {
      if (std::abs(vels[i]) > std::abs(maxs[i])) {
        printf("\033[31m[err]\033[0m: invalid generated profile\n");
        printf("vx=%f vy=%f vh=%f\n", p.vx, p.vy, p.vh);
        printf("wheel velocities:\n");
        show_vector(vels);
        broken = true;
        break;
      }
    }
    if (broken)
      break;
  }

  // TODO: clamp with vh acceleration
  double x = 0;
  double y = 0;
  double h = 0;
  double time = 0;
  for (auto &p : trajectory) {
    x += p.vx * (p.t - time);
    y += p.vy * (p.t - time);
    h += p.vh * (p.t - time);
    time = p.t;
  }
  printf("\033[34m[inf]\033[0m: integrated position error: %.2f %.2f %.2f\n",
         trajectory.back().x - x,
         trajectory.back().y - y,
         shorter_turn(h, trajectory.back().h));
  return 0;
}
