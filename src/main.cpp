#include "main.h"
#include "devices.hpp"

#include "subzerolib/api.hpp"

#include "pros/colors.hpp"
#include "pros/screen.hpp"
#include <memory>

namespace saturnine {
bool running = true;
};

// #define DEBUG

// TODO: test PID on the arm

std::unique_ptr<Filter> filter = nullptr;

// TODO: refactor, add "graph_module"?
void odom_disp_loop(void *ignore) {
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
  const double padding = 12;

  while (saturnine::running) {
    auto pose = odom->get_pose();
    past_points.push_back(pose.point());
    if (past_points.size() > 200) {
      past_points.erase(past_points.begin());
    }

    pros::screen::set_pen(pros::Color::black);
    pros::screen::fill_rect(graphx1 - padding,
                            graphy1 - padding,
                            graphx2 + padding,
                            graphy2 + padding);
    pros::screen::set_pen(pros::Color::gray);
    pros::screen::draw_line(graphx1, graphymid, graphx2,
                            graphymid); // x axis
    pros::screen::draw_line(graphxmid, graphy1, graphxmid,
                            graphy2); // y axis
    pros::screen::set_pen(pros::Color::light_gray);
    pros::screen::draw_rect(graphx1 - padding,
                            graphy1 - padding,
                            graphx2 + padding,
                            graphy2 + padding); // frame

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
    subzero::print(0, "filtered pos + vel");
    subzero::print(
        1, "(%6.3f, %6.3f) h: %5.0f", pose.x, pose.y, pose.heading());
    pose = odom->get_vel();
    subzero::print(
        2, "(%6.3f, %6.3f) h: %5.0f", pose.x, pose.y, pose.heading());
    subzero::print(3, "raw pos + vel");
    auto obj = dynamic_cast<KFOdometry *>(odom.get());
    pose = obj->get_raw_pose();
    subzero::print(
        4, "(%6.3f, %6.3f) h: %5.0f", pose.x, pose.y, pose.heading());
    pose = obj->get_raw_vel();
    subzero::print(
        5, "(%6.3f, %6.3f) h: %5.0f", pose.x, pose.y, pose.heading());
    // Eigen::VectorXd d = obj->get_covariance().diagonal();
    // subzero::print(6, "%f %f", d(0), d(1));
    // subzero::print(7, "%f %f", d(2), d(3));
    // subzero::print(8, "%f %f", d(4), d(5));

    pros::delay(10);
  }
}

void initialize() {
  subzero::set_log_area(0, 18, 480, 240);

  initialise_devices();

  pros::Task graphing_task{odom_disp_loop, nullptr, "odom display task"};
}

void disabled() {}

void competition_initialize() {}

void opcontrol() {
#ifdef DEBUG
  autonomous();
#endif
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  auto pose = odom->get_pose();
  if (std::isnan(pose.h))
    pose.h = 0.0;

#ifdef ROTATION_CONTROL_PID
  PIDF angle_pid(0.02, 0.0, 0.0008);
  double target_angle = pose.h;
#endif

  std::uint32_t prev_update = pros::millis();
  std::uint32_t *prev_update_ptr = &prev_update;

  while (saturnine::running) {
    // TODO: adjustments to increase accuracy along diagonals
    // model as a polar radial percentage of a rounded circle?
    double ctrl_x = master.get_analog(ANALOG_RIGHT_X) / 127.0;
    double ctrl_y = master.get_analog(ANALOG_RIGHT_Y) / 127.0;
    double ctrl_rx = master.get_analog(ANALOG_LEFT_X) / 127.0;
#ifdef ROTATION_CONTROL_PID
    double ctrl_ry = master.get_analog(ANALOG_LEFT_Y) / 127.0;
#endif

    pose = odom->get_pose();
    if (std::isnan(pose.h))
      pose.h = 0.0;
    auto vec = rotate_acw(ctrl_x, ctrl_y, pose.h);

#ifdef ROTATION_CONTROL_PID
    if (std::abs(ctrl_rx) < 0.2 && std::abs(ctrl_ry) < 0.2) {
      target_angle = pose.h;
    } else {
      target_angle = 90 - in_deg(atan2(ctrl_ry, ctrl_rx));
    }
    auto angle_err = shorter_turn(pose.h, target_angle);
    angle_pid.update(angle_err);
    if (std::abs(angle_err) > 1 &&
        std::abs(angle_pid.get_output()) > 0.3) { // anti jitter
      chassis->move(vec.x, vec.y, angle_pid.get_output());
    } else {
      chassis->move(vec.x, vec.y, 0.5 * angle_pid.get_output());
    }
#else
    chassis->move(vec.x, vec.y, 0.75 * ctrl_rx);
#endif

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      if (odom->is_enabled()) {
        odom->set_enabled(false);
      } else {
        odom->set_enabled(true);
      }
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
      odom->set_heading(0);

    // high update rate, as imu data comes in every 10 ms
    pros::Task::delay_until(prev_update_ptr, 10);
  }

  // garbage collection, good practice
  // free all memory
  //   no need in XChassis, everything is a smart pointer
  //   no need for imu, it's a smart pointer
  //   no need in GyroOdometry, everything is a smart pointer
  // delete pointers
}
