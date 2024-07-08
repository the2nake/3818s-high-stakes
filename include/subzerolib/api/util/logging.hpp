#pragma once

#include "pros/screen.hpp"
#include <string>

namespace subzero {
template <typename... Params>
void print(const int line, std::string msg, Params... args) {
  pros::screen::print(pros::E_TEXT_MEDIUM, line, msg.c_str(), args...);
}

template <typename... Params> void log(std::string msg, Params... args) {
  pros::screen::scroll_area(0, 180, 480, 240, 15);
  pros::screen::print(pros::E_TEXT_MEDIUM, 11, (msg + "%s").c_str(), args...,
                      "                                                 ");
}
}; // namespace subzero