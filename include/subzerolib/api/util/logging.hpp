#pragma once

#include "pros/screen.hpp"
#include <string>

namespace subzero {

extern int log_area_x1;
extern int log_area_y1;
extern int log_area_x2;
extern int log_area_y2;

void set_log_area(int x1, int y1, int x2, int y2);

template <typename... Params>
void print(const int line, std::string msg, Params... args) {
  pros::screen::print(pros::E_TEXT_MEDIUM, line, msg.c_str(), args...);
}

template <typename... Params> void log(std::string msg, Params... args) {
  pros::screen::scroll_area(log_area_x1, log_area_y1, log_area_x2, log_area_y2,
                            15);
  auto color = pros::screen::get_pen();
  pros::screen::set_pen(pros::Color::cornflower_blue);
  pros::screen::print(pros::E_TEXT_MEDIUM, 11, (msg + "%s").c_str(), args...,
                      "                                                 ");
  pros::screen::set_pen(color);
}

template <typename... Params> void error(std::string msg, Params... args) {
  pros::screen::scroll_area(log_area_x1, log_area_y1, log_area_x2, log_area_y2,
                            15);
  auto color = pros::screen::get_pen();
  pros::screen::set_pen(pros::Color::pink);
  pros::screen::print(pros::E_TEXT_MEDIUM, 11, (msg + "%s").c_str(), args...,
                      "                                                 ");
  pros::screen::set_pen(color);
}
}; // namespace subzero