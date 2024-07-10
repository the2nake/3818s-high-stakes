#include "subzerolib/api/util/logging.hpp"

namespace subzero {

int log_area_x1 = 0;
int log_area_y1 = 18;
int log_area_x2 = 480;
int log_area_y2 = 240;

void set_log_area(int x1, int y1, int x2, int y2) {
  subzero::log_area_x1 = x1;
  subzero::log_area_x2 = x2;
  subzero::log_area_y1 = y1;
  subzero::log_area_y2 = y2;
}

}; // namespace subzero