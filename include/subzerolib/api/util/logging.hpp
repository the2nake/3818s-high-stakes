#pragma once

#include "pros/screen.hpp"
#include <string>

namespace subzero {

extern int log_area_x1; // left bound
extern int log_area_y1; // top bound
extern int log_area_x2; // right bound
extern int log_area_y2; // bottom bound

/// @brief change the area of the logging
/// @param x1 left bound
/// @param y1 top bound
/// @param x2 right bound
/// @param y2 bottom bound
void set_log_area(int x1, int y1, int x2, int y2);

/// @brief print out a message
/// @tparam Params print parameters
/// @param line the line number
/// @param msg the message to pring
/// @param args the insertables
template <typename... Params>
inline void print(const int line, std::string msg, Params... args) {
  pros::screen::print(pros::E_TEXT_MEDIUM, line, msg.c_str(), args...);
}

/// @brief log a message
/// @tparam Params print parameters
/// @param msg the message to pring
/// @param args the insertables
template <typename... Params> inline void log(std::string msg, Params... args) {
  pros::screen::scroll_area(log_area_x1, log_area_y1, log_area_x2, log_area_y2,
                            15);
  auto color = pros::screen::get_pen();
  pros::screen::set_pen(pros::Color::cornflower_blue);
  pros::screen::print(pros::E_TEXT_MEDIUM, 11, (msg + "%s").c_str(), args...,
                      "                                                 ");
  pros::screen::set_pen(color);
}

/// @brief log an error
/// @tparam Params print parameters
/// @param msg the message to pring
/// @param args the insertables
template <typename... Params>
inline void error(std::string msg, Params... args) {
  pros::screen::scroll_area(log_area_x1, log_area_y1, log_area_x2, log_area_y2,
                            15);
  auto color = pros::screen::get_pen();
  pros::screen::set_pen(pros::Color::pink);
  pros::screen::print(pros::E_TEXT_MEDIUM, 11, (msg + "%s").c_str(), args...,
                      "                                                 ");
  pros::screen::set_pen(color);
}
}; // namespace subzero