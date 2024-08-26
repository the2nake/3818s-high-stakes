#include "subzerolib/api/geometry/trajectory-point.hpp"
#include "subzerolib/api/spline/catmull-rom.hpp"
#include "subzerolib/api/trajectory/motion-profile/trapezoidal-motion-profile.hpp"
#include "subzerolib/api/trajectory/spline-trajectory.hpp"
#include "subzerolib/api/util/math.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <sys/stat.h>
#include <sys/types.h>

void show_vector(std::vector<double> vector) {
  for (auto &i : vector) {
    printf("%f\n", i);
  }
}

class StarChassisKinematics : public Chassis {
public:
  StarChassisKinematics(double i_max_vel, double i_boost, double i_corner)
      : max_vel(std::abs(i_max_vel)), boost_radius(std::abs(i_boost)),
        corner_radius(std::abs(i_corner)) {}
  virtual void move(double x, double y, double r) override {}
  virtual void set_rot_pref(double irot_pref = 0.5) override {}

  double get_max_vel() override { return max_vel; }
  std::vector<double> get_wheel_max() override {
    return {0.5 * K_SQRT_2 * max_vel,
            max_vel,
            0.5 * K_SQRT_2 * max_vel,
            0.5 * K_SQRT_2 * max_vel,
            max_vel,
            0.5 * K_SQRT_2 * max_vel};
  }
  std::vector<double>
  get_wheel_vels(double vx, double vy, double v_ang) override {
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

int find_pose_index(std::vector<spline_point_s> &vec, pose_s pose) {
  double min_d = std::numeric_limits<double>::max();
  int return_index = 0;
  for (int i = 0; i < vec.size(); ++i) {
    double dist = pose.dist(vec[i]);
    if (dist < min_d) {
      return_index = i;
      min_d = dist;
    }
  }
  return return_index;
}

int main() {
  /*
  // check for unmet model constraints
  std::vector<double> maxs = kinematics.get_wheel_max();
  for (trajectory_point_s &p : trajectory) {
    auto loc = rotate_acw(p.vx, p.vy, p.h);
    auto vels = kinematics.get_wheel_vels(loc.x, loc.y, in_rad(p.vh));
    bool broken = false;
    for (int i = 0; i < vels.size(); ++i) {
      if (std::abs(vels[i]) > std::abs(maxs[i])) {
        printf("\033[31m[e]\033[0m: invalid generated profile\n");
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
         */

  // kinematics
  std::shared_ptr<Chassis> chassis{
      new StarChassisKinematics{1.73, 0.35, 0.37}
  };

  // set up a linear motion profile
  std::shared_ptr<TrapezoidalMotionProfile> lin_profile{
      new TrapezoidalMotionProfile{chassis->get_max_vel(), 6, 6}
  };
  lin_profile->set_resolution(0.005);

  // generate the curve using a catmull rom spline
  // std::vector<pose_s> ctrl = {
  //     pose_s{  0.0,  0.0,   0.0},
  //     pose_s{  0.4,  0.6,  45.0},
  //     pose_s{ -0.2,  0.6,  60.0},
  //     pose_s{-0.75, 0.75, -45.0}
  // };
  //

  std::vector<pose_s> ctrl = {
      pose_s{  0.0,  0.0,   0.0},
      pose_s{  0.5,  0.5,  45.0},
      pose_s{ -0.5,  1.0,  60.0},
      pose_s{-0.75, 0.75, -45.0}
  };

  std::shared_ptr<CatmullRomSpline> spline{new CatmullRomSpline{ctrl}};
  spline->pad_velocity({0.5, 0.5}, {-0.25, 0.25});

  auto gen =
      SplineTrajectory::Builder(SplineTrajectory::heading_mode_e::pose, 400)
          .with_spline(spline, ctrl)
          .with_motion_profile(lin_profile)
          .with_chassis(chassis)
          .build();

  gen->print();

  double x = 0;
  double y = 0;
  double h = 0;
  double time = 0;
  const double dt = 0.01;
  std::vector<trajectory_point_s> points;
  while (time + dt <= gen->get_duration()) {
    // BUG: get at time has issues with heading
    auto p = gen->get_at_time(time + dt);
    x += p.vx * dt;
    y += p.vy * dt;
    h += p.vh * dt;
    time += dt;
    points.push_back(p);
  }

  auto final = gen->get_at_distance(gen->get_length());
  printf("\033[34m[i]\033[0m: integrated position error: %.2f %.2f %.2f\n",
         final.x - x,
         final.y - y,
         shorter_turn(h, final.h));

  // output the sampled points into a file
  mkdir("test-output", 0777);
  std::fstream file;
  file.open("test-output/test-mp.txt", std::fstream::out);
  file.clear();

  for (auto &p : points) {
    file << "x " << p.t << " " << p.x << std::endl;
  }
  for (auto &p : points) {
    file << "vx " << p.t << " " << p.vx << std::endl;
  }
  for (auto &p : points) {
    file << "y " << p.t << " " << p.y << std::endl;
  }
  for (auto &p : points) {
    file << "vy " << p.t << " " << p.vy << std::endl;
  }
  for (auto &p : points) {
    file << "s " << p.t << " " << p.s << std::endl;
  }
  for (auto &p : points) {
    file << "v " << p.t << " " << std::hypot(p.vx, p.vy) << std::endl;
  }
  for (auto &p : points) {
    file << "h " << p.t << " " << p.h << std::endl;
  }
  for (auto &p : points) {
    file << "vh " << p.t << " " << p.vh << std::endl;
  }
  return 0;
}
