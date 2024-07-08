#pragma once

#include "pros/rtos.hpp"
#include <functional>

template <typename InputType> class AutoUpdater {
public:
  AutoUpdater(std::function<void(InputType)> iupdater,
              std::function<InputType()> igetter)
      : updater(std::move(iupdater)), getter(igetter) {}

  ~AutoUpdater() {
    stop();
  }

  void start(const int delay) {
    update_task = new pros::Task{[this, delay] {
      while (true) {
        uint32_t start = pros::millis();
        const auto ptr = &start;
        this->updater(this->getter());
        pros::Task::delay_until(ptr, delay);
      }
    }};
  }

  void stop() {
    if (update_task != nullptr) {
      update_task->remove();
      delete update_task;
      update_task = nullptr;
    }
  }

private:
  std::function<void(InputType)> updater = nullptr;
  std::function<InputType()> getter = nullptr;

  pros::Task *update_task = nullptr;
};
