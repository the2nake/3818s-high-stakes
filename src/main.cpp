#include "main.h"

#include "pros/colors.hpp"
#include "subzerolib/api.hpp"
#include "subzerolib/api/chassis/star-chassis.hpp"
#include "subzerolib/api/control/holo-chassis-pid.hpp"
#include "subzerolib/api/control/pure-pursuit.hpp"
#include "subzerolib/api/odometry/imu_odometry.hpp"
#include "subzerolib/api/odometry/odometry.hpp"
#include "subzerolib/api/sensors/abstract_encoder.hpp"
#include "subzerolib/api/sensors/abstract_gyro.hpp"
#include "subzerolib/api/spline/catmull-rom.hpp"
#include "subzerolib/api/util/helper.hpp"
#include "subzerolib/api/util/logging.hpp"
#include "subzerolib/api/util/math.hpp"

#include "pros/abstract_motor.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <memory>

namespace saturnine {
bool running = true;
};

// TODO: test PID on the arm

std::unique_ptr<pros::Motor> fl(new pros::Motor(-1, pros::v5::MotorGears::green,
                                                pros::v5::MotorUnits::deg));
std::unique_ptr<pros::Motor> fr(new pros::Motor(10, pros::v5::MotorGears::green,
                                                pros::v5::MotorUnits::deg));
std::unique_ptr<pros::Motor> ml(new pros::Motor(-12,
                                                pros::v5::MotorGears::green,
                                                pros::v5::MotorUnits::deg));
std::unique_ptr<pros::Motor> mr(new pros::Motor(19, pros::v5::MotorGears::green,
                                                pros::v5::MotorUnits::deg));
std::unique_ptr<pros::Motor> br(new pros::Motor(20, pros::v5::MotorGears::green,
                                                pros::v5::MotorUnits::deg));
std::unique_ptr<pros::Motor> bl(new pros::Motor(-11,
                                                pros::v5::MotorGears::green,
                                                pros::v5::MotorUnits::deg));
std::shared_ptr<StarChassis> chassis = nullptr;
std::shared_ptr<AbstractGyro> imu{new AbstractImuGyro(9)};

// TODO: make ports.h
std::shared_ptr<AbstractEncoder> odom_x{new AbstractRotationEncoder(6, true)};
std::shared_ptr<AbstractEncoder> odom_y{new AbstractRotationEncoder(7, true)};
std::shared_ptr<Odometry> odom = nullptr;

void odom_graph_loop(void *ignore) {
  std::vector<point_s> past_points;
  const int graphx1 = 330;
  const int graphy1 = 30;
  const int graphx2 = 450;
  const int graphy2 = 150;
  const int graphxmid = (graphx1 + graphx2) / 2.0;
  const int graphymid = (graphy1 + graphy2) / 2.0;
  const int graphw = std::abs(graphx1 - graphx2) / 2.0;
  const int graphh = std::abs(graphy1 - graphy2) / 2.0;

  const double max_x = 0.5;
  const double max_y = 0.5;

  while (saturnine::running) {
    past_points.push_back(odom->get_pose().point());
    if (past_points.size() > 200) {
      past_points.erase(past_points.begin());
    }

    pros::screen::set_pen(pros::Color::black);
    pros::screen::fill_rect(graphx1, graphy1, graphx2, graphy2);
    pros::screen::set_pen(pros::Color::gray);
    pros::screen::draw_line(graphx1, graphymid, graphx2,
                            graphymid); // x axis
    pros::screen::draw_line(graphxmid, graphy1, graphxmid,
                            graphy2); // y axis
    pros::screen::set_pen(pros::Color::light_gray);
    pros::screen::draw_rect(graphx1 - 12, graphy1 - 12, graphx2 + 12,
                            graphy2 + 12); // frame

    for (int i = 0; i < past_points.size(); ++i) {
      if (i >= past_points.size() * 0.75) {
        pros::screen::set_pen(pros::Color::white);
      } else if (i >= past_points.size() * 0.5) {
        pros::screen::set_pen(pros::Color::light_gray);
      } else if (i >= past_points.size() * 0.25) {
        pros::screen::set_pen(pros::Color::dark_gray);
      } else {
        pros::screen::set_pen(pros::Color::gray);
      }
      point_s pixel_point{graphxmid + past_points[i].x * graphw / (2.0 * max_x),
                          graphymid -
                              past_points[i].y * graphh / (max_y * 2.0)};

      if (pixel_point.x > graphx2 || pixel_point.x < graphx1 ||
          pixel_point.y > graphy2 || pixel_point.y < graphy1) {
        continue;
      }

      if (i == past_points.size() - 1) {
        pros::screen::draw_circle(pixel_point.x, pixel_point.y, 1);
        pros::screen::draw_pixel(pixel_point.x, pixel_point.y + 3);
        pros::screen::draw_pixel(pixel_point.x, pixel_point.y - 3);
        pros::screen::draw_pixel(pixel_point.x + 3, pixel_point.y);
        pros::screen::draw_pixel(pixel_point.x - 3, pixel_point.y);
        pros::screen::set_pen(pros::Color::black);
        pros::screen::draw_pixel(pixel_point.x, pixel_point.y);
      } else {
        pros::screen::draw_pixel(pixel_point.x, pixel_point.y);
      }
    }
    pros::screen::set_pen(pros::Color::white);
    pros::delay(10);
  }
}

void initialize() {
  auto imus = pros::Imu::get_all_devices();
  for (auto device : imus) {
    subzero::log("[info]: resetting imu on port %d", device.get_port());
    device.reset();
  }

  for (auto device : imus) {
    while (device.is_calibrating()) {
      pros::delay(100);
    }
    subzero::log("[info]: imu on port %d ready", device.get_port());
  }

  pros::delay(250);

  chassis =
      StarChassis::Builder()
          .with_motors(StarChassis::motor_position_e::front_left, std::move(fl))
          .with_motors(StarChassis::motor_position_e::front_right,
                       std::move(fr))
          .with_motors(StarChassis::motor_position_e::boost_left, std::move(ml))
          .with_motors(StarChassis::motor_position_e::boost_right,
                       std::move(mr))
          .with_motors(StarChassis::motor_position_e::back_left, std::move(bl))
          .with_motors(StarChassis::motor_position_e::back_right, std::move(br))
          .with_geometry(0.35, 0.37)
          .with_rot_pref(0.3)
          .build();

  odom =
      ImuOdometry::Builder()
          .with_gyro(imu)
          .with_x_enc(odom_x, Odometry::encoder_conf_s(-0.045, 0.160 / 360.0))
          .with_y_enc(odom_y, Odometry::encoder_conf_s(0.09, 0.160 / 360.0))
          .build();
  odom->auto_update(10);

  pros::Task graphing_task{odom_graph_loop, nullptr, "odom graphing task"};
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  auto auton_start_time = pros::millis();
  std::shared_ptr<HoloChassisPID> controller =
      HoloChassisPID::Builder()
          .with_chassis(chassis)
          .with_odom(odom)
          .with_pid(HoloChassisPID::pid_dimension_e::x, 1.7, 0.0001, 0.09)
          .with_pid(HoloChassisPID::pid_dimension_e::y, 1.7, 0.0001, 0.09)
          .with_pid(HoloChassisPID::pid_dimension_e::r, 0.01, 0.0, 0)
          .build();
  std::shared_ptr<ExitCondition<double>> cond{
      new ExitCondition<double>{{0, 0.02}, 200}};
  const pose_s target{0.3, 0.3, 270};
  odom->set_position(0.0, 0.0);
  odom->set_heading(0.0);

  AutoUpdater<double> updater(
      [&cond](double val) { cond->update(val); },
      [&, target]() -> double { return odom->get_pose().dist(target); });
  updater.start(10);
  while (!cond->is_met() && pros::millis() - auton_start_time < 14900) {
    controller->approach_pose(target);
    pros::delay(10);
  }
  updater.stop();
  subzero::log("[info]: pid to (%.2f, %.2f) @ %.0f done", target.x, target.y,
               target.h);
  /*
  // TODO: tune integral
  PurePursuitController pp(controller, odom, std::move(cond));
  std::vector<pose_s> ctrl = {
      {0.0, 0.0, 0.0}, {0.4, 0.6, 45.0}, {-0.2, 0.6, 60.0}, {-1.0, 1.0, -45.0}};
  CatmullRomSpline spline(ctrl);
  spline.pad_velocity({0.5, 0.5}, {-0.25, 0.25});
  auto spline_points = spline.sample(200);
  std::vector<pose_s> waypoints(spline_points.size());
  transform(spline_points.begin(), spline_points.end(), waypoints.begin(),
            [](point_s point) -> pose_s { return pose_s{point}; });

  // TODO: fix bug, data abort exception, path following seems to start
  // correctly
  // TODO: rewrite pure pursuit? it's very short
  pp.follow(waypoints, 0.1);
  */
}

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  while (saturnine::running) {
    // Arcade control scheme
    // FIXME: may need adjustments to increase accuracy along diagonals
    // model as a polar radial percentage of a rounded circle
    double x = master.get_analog(ANALOG_RIGHT_X) / 127.0;
    double y = master.get_analog(ANALOG_RIGHT_Y) / 127.0;
    double r = master.get_analog(ANALOG_LEFT_X) / 127.0;
    auto pose = odom->get_pose();

    if (std::isnan(pose.h)) {
      pose.h = 0.0;
    }

    auto vec = rotate_acw(x, y, pose.h);
    chassis->move(vec.x, vec.y, 0.75 * r);

    // TODO: chassis angle correction
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      if (odom->is_enabled()) {
        odom->set_enabled(false);
      } else {
        odom->set_enabled(true);
      }
    }
    subzero::print(0, "(%.2f, %.2f) h: %.0f", pose.x, pose.y, pose.heading());

    pros::delay(10); // high update rate, as imu data comes in every 10 ms
  }

  // garbage collection, good practice
  // free all memory
  //   no need in XChassis, everything is a smart pointer
  //   no need for imu, it's a smart pointer
  //   no need in ImuOdometry, everything is a smart pointer
  // delete pointers
}