#include "main.h"
#include "ports.h"

#include "subzerolib/api.hpp"

#include "pros/abstract_motor.hpp"
#include "pros/colors.hpp"
#include "pros/screen.hpp"
#include <algorithm>
#include <memory>

namespace saturnine {
bool running = true;
};

// TODO: test PID on the arm

std::unique_ptr<pros::Motor> fl(new pros::Motor(-DRIVE_FL_PORT,
                                                pros::v5::MotorGears::green,
                                                pros::v5::MotorUnits::deg));
std::unique_ptr<pros::Motor> fr(new pros::Motor(DRIVE_FR_PORT,
                                                pros::v5::MotorGears::green,
                                                pros::v5::MotorUnits::deg));
std::unique_ptr<pros::Motor> ml(new pros::Motor(-DRIVE_ML_PORT,
                                                pros::v5::MotorGears::green,
                                                pros::v5::MotorUnits::deg));
std::unique_ptr<pros::Motor> mr(new pros::Motor(DRIVE_MR_PORT,
                                                pros::v5::MotorGears::green,
                                                pros::v5::MotorUnits::deg));
std::unique_ptr<pros::Motor> br(new pros::Motor(DRIVE_BR_PORT,
                                                pros::v5::MotorGears::green,
                                                pros::v5::MotorUnits::deg));
std::unique_ptr<pros::Motor> bl(new pros::Motor(-DRIVE_BL_PORT,
                                                pros::v5::MotorGears::green,
                                                pros::v5::MotorUnits::deg));
std::shared_ptr<StarChassis> chassis = nullptr;
std::shared_ptr<AbstractGyro> imu2{new AbstractImuGyro(IMU2_PORT)};

std::shared_ptr<AbstractEncoder> odom_x{
    new AbstractRotationEncoder(PORT_X_ENC, true)};
std::shared_ptr<AbstractEncoder> odom_y{
    new AbstractRotationEncoder(PORT_Y_ENC, true)};
std::shared_ptr<Odometry> odom = nullptr;

// TODO: refactor, add "graph_module"?
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
    pros::screen::fill_rect(graphx1 - 12, graphy1 - 12, graphx2 + 12,
                            graphy2 + 12);
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
  subzero::set_log_area(0, 18, 480, 240);

  auto imus = pros::Imu::get_all_devices();
  for (auto device : imus) {
    subzero::log("[i]: resetting imu on port %d", device.get_port());
    device.reset();
  }

  for (auto device : imus) {
    while (device.is_calibrating()) {
      pros::delay(100);
    }
    subzero::log("[i]: imu on port %d ready", device.get_port());
  }

  pros::delay(500);

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
          .with_gyro(imu2)
          .with_x_enc(odom_x, Odometry::encoder_conf_s(-0.045, 0.160 / 360.0))
          .with_y_enc(odom_y, Odometry::encoder_conf_s(0.09, 0.160 / 360.0))
          .build();
  odom->auto_update(10);

  pros::Task graphing_task{odom_graph_loop, nullptr, "odom graphing task"};
}

void disabled() {}

void competition_initialize() {}

void go_to(std::shared_ptr<ChassisController> controller, pose_s target) {
  std::shared_ptr<ExitCondition<double>> cond{
      new ExitCondition<double>{{0, 0.02}, 200}};
  AutoUpdater<double> updater(
      [cond](double val) { cond->update(val); },
      [target]() -> double { return odom->get_pose().dist(target); });
  updater.start(10);
  while (!cond->is_met()) {
    controller->approach_pose(target);
    pros::delay(10);
  }
  controller->brake();
  updater.stop(); // TODO: stop when out of scope
  subzero::log("[i]: pid to (%.2f, %.2f) @ %.0f done", target.x, target.y,
               target.h);
}

void autonomous() {
  auto auton_start_time = pros::millis();
  std::shared_ptr<HoloChassisPID> controller =
      HoloChassisPID::Builder()
          .with_chassis(chassis)
          .with_odom(odom)
          .with_pid(HoloChassisPID::pid_dimension_e::x, 3.6, 0, 0.42)
          .with_pid(HoloChassisPID::pid_dimension_e::y, 3.6, 0, 0.42)
          .with_pid(HoloChassisPID::pid_dimension_e::r, 0.015, 0.0, 0.0008)
          .build();
  odom->set_position(0.0, 0.0);
  odom->set_heading(0.0);

  // go_to(controller, {0.3, 0.3, 270});
  // go_to(controller, {-0.6, 0.5, 315});

  std::shared_ptr<ExitCondition<double>> cond{
      new ExitCondition<double>{{0, 0.02}, 200}};
  PurePursuitController pp(controller, odom, std::move(cond));

  std::vector<pose_s> ctrl = {{0.0, 0.0, 0.0},
                              {0.4, 0.6, 45.0},
                              {-0.2, 0.6, 60.0},
                              {-0.75, 0.75, -45.0}};
  CatmullRomSpline spline(ctrl);
  spline.pad_velocity({0.5, 0.5}, {-0.25, 0.25});
  auto spline_points = spline.sample(200);
  std::vector<pose_s> waypoints(spline_points.size());
  transform(spline_points.begin(), spline_points.end(), waypoints.begin(),
            [](point_s point) -> pose_s { return pose_s{point, 0.0}; });
  pp.follow(waypoints, 0.4);
}

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  PIDF angle_pid(0.02, 0.0, 0.0008);
  auto pose = odom->get_pose();
  if (std::isnan(pose.h)) {
    pose.h = 0.0;
  }
  double target_angle = pose.h;

  while (saturnine::running) {
    // TODO: adjustments to increase accuracy along diagonals
    // model as a polar radial percentage of a rounded circle?
    double ctrl_x = master.get_analog(ANALOG_RIGHT_X) / 127.0;
    double ctrl_y = master.get_analog(ANALOG_RIGHT_Y) / 127.0;
    double ctrl_rx = master.get_analog(ANALOG_LEFT_X) / 127.0;
    double ctrl_ry = master.get_analog(ANALOG_LEFT_Y) / 127.0;

    pose = odom->get_pose();
    if (std::isnan(pose.h)) {
      pose.h = 0.0;
    }

    if (std::abs(ctrl_rx) < 0.2 && std::abs(ctrl_ry) < 0.2) {
      target_angle = pose.h;
    } else {
      target_angle = 90 - in_deg(atan2(ctrl_ry, ctrl_rx));
    }
    auto angle_err = shorter_turn(pose.h, target_angle);
    angle_pid.update(angle_err);
    auto vec = rotate_acw(ctrl_x, ctrl_y, pose.h);
    if (std::abs(angle_err) > 1 &&
        std::abs(angle_pid.get_output()) > 0.3) { // anti jitter
      chassis->move(vec.x, vec.y, angle_pid.get_output());
    } else {
      chassis->move(vec.x, vec.y, 0.5 * angle_pid.get_output());
    }

    // chassis->move(vec.x, vec.y, 0.75 * ctrl_rx);

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