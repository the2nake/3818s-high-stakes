#pragma once

#include "pros/rtos.hpp"
#include <atomic>
#include <functional>

// concept kinda sorta stolen from lemlib
// TODO: test ExitCondition auto update

template <typename T> class ExitCondition {
public:
  template <typename range_t> struct range_s {
    range_s(range_t imin, range_t imax, bool iinclusive = true)
        : inclusive(iinclusive) {
      if (imin > imax) {
        max = imin;
        min = imax;
      } else {
        min = imin;
        max = imax;
      }
    }
    range_t min;
    range_t max;
    bool inclusive;

    bool contains(range_t val) {
      if (inclusive) {
        return min <= val && max >= val;
      } else {
        return min < val && max > val;
      }
    }
  };

  ExitCondition(range_s<T> irange, int imin_ms = 500)
      : range(irange), min_ms(imin_ms) {
    last_update = pros::millis();
  }

  ~ExitCondition() {
    stop_updating();
    pros::delay(20);
    delete this->update_task;
    update_task = nullptr;
  }

  void update(T input) {
    uint32_t now = pros::millis();
    if (range.contains(input)) {
      met_duration += now - last_update;
    } else {
      met_duration = 0;
    }
    last_update = now;
  }

  bool is_met() { return met_duration.load() >= min_ms; }

  void auto_update(std::function<double()> igetter, int iupdate_delay) {
    getter = igetter;
    update_delay = iupdate_delay;
    uint32_t prev_ts = pros::millis();
    const auto ptr = &prev_ts;

    update_task = new pros::Task([&, this] {
      while (!pros::Task::notify_take(true, 0)) {
        this->update(getter());
        pros::Task::delay_until(ptr, update_delay);
      }
    });
  }

  void stop_updating() {
    if (this->update_task != nullptr) {
      this->update_task->notify();
    }
  }

private:
  std::function<T()> getter = nullptr;
  pros::Task *update_task = nullptr;
  int update_delay = 10;

  range_s<T> range;
  int min_ms;
  std::atomic<int> met_duration = 0;
  uint32_t last_update = pros::millis();
};