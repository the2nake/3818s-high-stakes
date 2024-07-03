#pragma once

#include <atomic>
#include <cmath>

class PIDF {
public:
  // why would you ever need to change constants idk
  PIDF(double ikp, double iki, double ikd, double iff = 0)
      : kp(ikp), ki(iki), kd(ikd), ff(iff) {}
  void reset();
  void set_ff(double iff) { ff = iff; }

  void update(double error);
  double get_output() { return output.load(); }

private:
  std::atomic<double> output;
  const double kp;
  const double ki;
  const double kd;
  double ff = 0.0;

  std::atomic<double> prev_err = std::nan("");
  std::atomic<double> total_err = 0.0;

  uint32_t last_update;
};
