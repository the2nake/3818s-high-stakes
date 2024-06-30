#include "subzerolib/api/trajectory/motion-profile/trapezoidal-motion-profile.hpp"

int main() {
  TrapezoidalMotionProfile trap{10, 20, 20};
  trap.generate(30);
  trap.print();
  return 0;
}